#!/usr/bin/env python3
"""Overhead AprilTag 36h11 timing; does not command the robot.

Unlike a short-term angular-velocity prediction, this tracker learns the
period from observed crossings of the chosen PICK angle. This makes a
repeatable one-shot pickup clock for a near-constant-speed turntable.

    ros2 run turntable_pick vision_timer --camera 0 --lead 17

Click the turntable center, then the PICK point on the tag's orbit.
Press 'r' to recalibrate, 'q' to stop. Camera resolution/position must be
unchanged when reusing calibration coordinates.
"""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import statistics
import time

TAU = 2.0 * math.pi


def signed_wrap(value):
    return (value + math.pi) % TAU - math.pi


def angle_from(center, point):
    return math.atan2(center[1] - point[1], point[0] - center[0])


def crossing_time(prev_t, prev_angle, now, now_angle, target_angle, omega_sign):
    """Interpolate a crossing of the pickup ray, or return None.

    Input angles are continuously unwrapped. Does not tolerate big frame gaps.
    """
    if omega_sign > 0:
        k = math.floor((prev_angle - target_angle) / TAU) + 1
        crossing = target_angle + TAU * k
        if not prev_angle < crossing <= now_angle:
            return None
    else:
        k = math.ceil((prev_angle - target_angle) / TAU) - 1
        crossing = target_angle + TAU * k
        if not now_angle <= crossing < prev_angle:
            return None
    fraction = (crossing - prev_angle) / (now_angle - prev_angle)
    return prev_t + (now - prev_t) * fraction


class RotationTracker:
    """Estimate turntable period from successive PICK crossing timestamps."""
    def __init__(self, center, pickup):
        self.center, self.pickup = center, pickup
        self.last_t = None
        self.last_angle = None
        self.unwrapped = None
        self.samples = deque(maxlen=150)
        self.crossings = deque(maxlen=8)
        self.last_valid_t = None
        self.period = None
        self.predicted_next = None
        self.last_prediction_error = None
        self.direction = None

    def update(self, now, xy):
        """Return a crossing timestamp if a crossing was observed now."""
        angle = angle_from(self.center, xy)
        if self.last_t is None or now <= self.last_t or now - self.last_t > 0.45:
            # Reacquiring after missed frames: do not invent a crossing.
            self.last_t, self.last_angle = now, angle
            self.unwrapped = angle
            self.samples.clear()
            self.samples.append((now, self.unwrapped))
            self.last_valid_t = now
            return None

        delta = signed_wrap(angle - self.last_angle)
        dt = now - self.last_t
        if abs(delta) > 0.25 or abs(delta) / dt > 1.0:
            # Spurious detection / bad calibration; restart phase tracking.
            self.last_t, self.last_angle = now, angle
            self.unwrapped = angle
            self.samples.clear()
            self.samples.append((now, self.unwrapped))
            self.last_valid_t = now
            return None

        previous_unwrapped = self.unwrapped
        self.unwrapped += delta
        previous_t = self.last_t
        self.last_t, self.last_angle = now, angle
        self.samples.append((now, self.unwrapped))
        self.last_valid_t = now
        while self.samples and now - self.samples[0][0] > 6.0:
            self.samples.popleft()

        if len(self.samples) < 8 or self.samples[-1][0] - self.samples[0][0] < 2.5:
            return None
        t0, a0 = self.samples[0]
        slope = (self.unwrapped - a0) / (now - t0)
        if not 0.12 <= abs(slope) <= 0.4:
            return None  # roughly 16–52 seconds per turn
        self.direction = 1 if slope > 0 else -1

        event_t = crossing_time(previous_t, previous_unwrapped,
                                now, self.unwrapped,
                                angle_from(self.center, self.pickup),
                                self.direction)
        if event_t is None:
            return None
        # Avoid repeat/camera false hits.
        if self.crossings and event_t - self.crossings[-1] < 10.0:
            return None

        if self.predicted_next is not None:
            self.last_prediction_error = event_t - self.predicted_next
        self.crossings.append(event_t)
        if len(self.crossings) >= 2:
            differences = [b - a for a, b in zip(self.crossings, list(self.crossings)[1:])]
            # Reject a period after a missed tag crossing or a bad frame.
            valid = [d for d in differences if 20.0 < d < 40.0]
            if valid:
                self.period = statistics.median(valid[-4:])
                self.predicted_next = event_t + self.period
            else:
                self.period = None
                self.predicted_next = None
        return event_t

    def is_fresh(self, now):
        return self.last_valid_t is not None and now - self.last_valid_t < 0.6


class CameraMonitor:
    def __init__(self, camera_index=0, calibration='vision_calibration.json', tag_id=0):
        try:
            import cv2
        except ImportError as e:
            raise RuntimeError('OpenCV missing: install opencv-contrib-python') from e
        if not hasattr(cv2, 'aruco') or not hasattr(cv2.aruco, 'DICT_APRILTAG_36h11'):
            raise RuntimeError('This OpenCV installation lacks AprilTag 36h11 detection')
        self.cv2 = cv2
        self.tag_id = tag_id
        self.calibration = Path(calibration)
        self.center, self.pickup = None, None
        if self.calibration.is_file():
            data = json.loads(self.calibration.read_text())
            self.center = tuple(data['center'])
            self.pickup = tuple(data['pickup'])
            print(f'Loaded calibration from {self.calibration} (press r to reset)')
        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            raise RuntimeError(f'Cannot open camera {camera_index} on this computer')
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        if hasattr(cv2.aruco, 'ArucoDetector'):
            self.detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
            self.detect = lambda im: self.detector.detectMarkers(im)
        else:
            parameters = cv2.aruco.DetectorParameters_create()
            self.detect = lambda im: cv2.aruco.detectMarkers(im, dictionary, parameters=parameters)
        self.tracker = None
        if self.center and self.pickup:
            self.tracker = RotationTracker(self.center, self.pickup)
        self.window = 'AprilTag vision timing (no robot control)'
        cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.window, self.on_mouse)
        self.flash_until = 0.0
        self.fired_prediction = None
        self.frame_shape = None
        print('Click TURNTABLE CENTER, then PICK point on tag path. Press r to reset; q to quit.')

    def on_mouse(self, event, x, y, flags, param):
        if event != self.cv2.EVENT_LBUTTONDOWN:
            return
        if self.center is None:
            self.center = (x, y)
            print(f'Center set: {self.center}; click PICK point')
        else:
            self.pickup = (x, y)
            self.tracker = RotationTracker(self.center, self.pickup)
            self.fired_prediction = None
            self.calibration.write_text(json.dumps({'center': self.center, 'pickup': self.pickup}, indent=2))
            print(f'PICK set: {self.pickup}; saved to {self.calibration}')

    def reset(self):
        self.center = self.pickup = self.tracker = self.fired_prediction = None
        if self.calibration.is_file():
            self.calibration.unlink()
        print('Calibration cleared; click center then PICK point')

    def draw_text(self, frame, lines, start_y=30):
        cv2 = self.cv2
        for line in lines:
            cv2.putText(frame, line, (12, start_y), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (255, 255, 255), 3)
            cv2.putText(frame, line, (12, start_y), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 0, 0), 1)
            start_y += 28

    def next_frame(self, lead=17.0):
        cv2 = self.cv2
        ok, frame = self.cap.read()
        if not ok:
            raise RuntimeError('Camera frame read failed')
        now = time.monotonic()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detect(gray)
        tag = None
        if ids is not None:
            for quad, ident in zip(corners, ids.flatten()):
                if int(ident) == self.tag_id:
                    import numpy as np
                    pts = np.asarray(quad).reshape(4, 2)
                    tag = tuple(np.mean(pts, axis=0))
                    cv2.polylines(frame, [pts.astype('int32')], True, (0, 220, 0), 2)
                    cv2.circle(frame, tuple(map(int, tag)), 4, (0, 220, 0), -1)
                    break
        crossing = None
        prediction = None
        if self.center is not None and self.pickup is not None:
            center, pickup = tuple(map(int, self.center)), tuple(map(int, self.pickup))
            radius = int(math.dist(center, pickup))
            cv2.circle(frame, center, max(radius, 1), (255, 150, 30), 1)
            cv2.circle(frame, center, 5, (255, 150, 30), -1)
            cv2.circle(frame, pickup, 8, (0, 0, 255), 2)
            cv2.putText(frame, 'PICK', (pickup[0] + 10, pickup[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            if tag is not None and self.tracker is not None:
                crossing = self.tracker.update(now, tag)
                if crossing is not None:
                    print(f'PICK CROSSING at t={crossing:.3f}', flush=True)
                    if self.tracker.last_prediction_error is not None:
                        print('  Previous prediction error: '
                              f'{self.tracker.last_prediction_error:+.3f} s', flush=True)
                    if self.tracker.period:
                        print(f'  Measured full-revolution period: {self.tracker.period:.3f} s',
                              flush=True)
            if self.tracker and self.tracker.period:
                prediction = self.tracker.predicted_next
                # The prediction anchors at a measured crossing, not an extrapolation
                # from a few frames; therefore the period estimate is more stable.
                if self.tracker.direction is not None:
                    # Show where START would be at this speed.
                    th_pick = angle_from(center, pickup)
                    th_start = th_pick - (TAU / self.tracker.period) * self.tracker.direction * lead
                    marker_xy = (int(center[0] + radius * math.cos(th_start)),
                                 int(center[1] - radius * math.sin(th_start)))
                    cv2.circle(frame, marker_xy, 8, (0, 255, 255), 2)
                    cv2.putText(frame, 'START', (marker_xy[0] + 10, marker_xy[1] + 13),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                age = prediction - now
                lines = [f'Period {self.tracker.period:.2f}s | {60/self.tracker.period:.2f} RPM',
                         f'Next PICK in {age:.2f}s | ARM lead {lead:.1f}s']
            else:
                lines = ['LEARNING PERIOD: wait for 2 PICK crossings (~1 revolution)']
            if tag is None:
                lines.append('TAG NOT DETECTED')
        else:
            lines = ['CLICK CENTER' if self.center is None else 'CLICK PICK POINT']

        # Fire at most once per cycle, only if fresh and prediction is still in the future.
        triggered = False
        if (self.tracker is not None and prediction is not None and
                self.tracker.is_fresh(now) and 0 < prediction - now <= lead and
                self.fired_prediction != prediction):
            # Skip startup/late trigger: require event occurred very close to crossing threshold.
            if abs((prediction - now) - lead) <= 0.35:
                triggered = True
                self.fired_prediction = prediction
                self.flash_until = now + 1.5
                print(f'>>> START ARM NOW at t={now:.3f}; '
                      f'predicted PICK at t={prediction:.3f}; '
                      f'ETA={prediction-now:.3f}s', flush=True)
        if now < self.flash_until:
            cv2.rectangle(frame, (0, 0), (frame.shape[1], 75), (0, 110, 0), -1)
            self.draw_text(frame, ['START ARM NOW (TIMING ONLY)'], 52)
            self.draw_text(frame, lines, 105)
        else:
            self.draw_text(frame, lines)
        cv2.putText(frame, 'click center/PICK | r reset | q quit',
                    (10, frame.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX,
                    0.52, (230, 230, 230), 1)
        cv2.imshow(self.window, frame)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            raise KeyboardInterrupt
        if key == ord('r'):
            self.reset()
        return now, crossing, prediction, triggered

    def get_robust_reference(self, min_crossings=3, max_wait=240.0):
        """Wait until >=min_crossings measured; return last crossing + period.

        Safe to pass through normal Kinova scheduling. This one-shot clock is
        not live visual servoing: after this method returns, it stops tracking.
        """
        started = time.monotonic()
        while time.monotonic() - started < max_wait:
            self.next_frame()
            if (self.tracker and len(self.tracker.crossings) >= min_crossings
                    and self.tracker.period and self.tracker.is_fresh(time.monotonic())):
                last = self.tracker.crossings[-1]
                period = self.tracker.period
                if any(not 20.0 < (b-a) < 40.0 for a, b in zip(list(self.tracker.crossings)[:-1], list(self.tracker.crossings)[1:])):
                    continue
                if last + period - time.monotonic() > 18.0:
                    print(f'VISION REF: measured period {period:.3f}s; '
                          f'next PICK in {last+period-time.monotonic():.2f}s')
                    return last, period
        raise RuntimeError('Camera did not establish reliable PICK crossings within time limit')

    def close(self):
        self.cap.release()
        self.cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera', type=int, default=0)
    parser.add_argument('--lead', type=float, default=17.0,
                        help='Seconds before actual expected PICK to show START (default 17)')
    parser.add_argument('--calibration', default='vision_calibration.json')
    parser.add_argument('--tag-id', type=int, default=0)
    args = parser.parse_args()
    if args.lead <= 0:
        parser.error('--lead must be positive')
    monitor = CameraMonitor(args.camera, args.calibration, args.tag_id)
    print('Camera-only preview. NO robot commands will be sent.')
    try:
        while True:
            monitor.next_frame(args.lead)
    except KeyboardInterrupt:
        print('Stopped camera timer.')
    finally:
        monitor.close()


if __name__ == '__main__':
    main()
