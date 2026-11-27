#!/usr/bin/env python3
"""AprilTag camera timing (36h11, ID 0); never sends robot commands.

The calibration stores the turntable center, PICK position, and learned
period in vision_calibration.json. A new process still observes a real PICK
crossing to establish *current phase*; stored speed alone cannot tell where
an already-spinning block is.
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
    """Interpolate the moment the tag passes the calibrated PICK ray."""
    if omega_sign > 0:
        k = math.floor((prev_angle - target_angle) / TAU) + 1
        target = target_angle + TAU * k
        if not prev_angle < target <= now_angle:
            return None
    else:
        k = math.ceil((prev_angle - target_angle) / TAU) - 1
        target = target_angle + TAU * k
        if not now_angle <= target < prev_angle:
            return None
    return prev_t + (now - prev_t) * (target - prev_angle) / (now_angle - prev_angle)


class RotationTracker:
    def __init__(self, center, pickup, cached_period=None):
        self.center, self.pickup = center, pickup
        self.last_t = None
        self.last_angle = None
        self.unwrapped = None
        self.samples = deque(maxlen=150)
        self.crossings = deque(maxlen=8)
        self.last_valid_t = None
        self.period = cached_period
        self.predicted_next = None
        self.last_prediction_error = None
        self.direction = None
        self.stable_period = False

    def update(self, now, xy):
        angle = angle_from(self.center, xy)
        if self.last_t is None or now <= self.last_t or now - self.last_t > 0.45:
            # On reacquisition do not invent a crossing or a whole revolution.
            self.last_t = now
            self.last_angle = self.unwrapped = angle
            self.samples.clear()
            self.samples.append((now, angle))
            self.last_valid_t = now
            return None

        delta = signed_wrap(angle - self.last_angle)
        dt = now - self.last_t
        if abs(delta) > 0.25 or abs(delta) / dt > 1.0:
            self.last_t = now
            self.last_angle = self.unwrapped = angle
            self.samples.clear()
            self.samples.append((now, angle))
            self.last_valid_t = now
            return None

        old_unwrapped, old_t = self.unwrapped, self.last_t
        self.unwrapped += delta
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
            return None
        self.direction = 1 if slope > 0 else -1
        event_t = crossing_time(old_t, old_unwrapped, now, self.unwrapped,
                                angle_from(self.center, self.pickup), self.direction)
        if event_t is None:
            return None
        if self.crossings and event_t - self.crossings[-1] < 10.0:
            return None

        if self.predicted_next is not None:
            self.last_prediction_error = event_t - self.predicted_next
        self.crossings.append(event_t)
        intervals = [b - a for a, b in zip(list(self.crossings)[:-1], list(self.crossings)[1:])]
        # Only the most RECENT *consecutive* valid intervals count. A missed
        # earlier revolution must not permanently block readiness.
        recent = intervals[-3:]
        if len(recent) >= 2 and all(20.0 < d < 40.0 for d in recent[-2:]):
            if abs(recent[-1] - recent[-2]) < 0.35:
                self.period = statistics.median(recent[-2:])
                self.stable_period = True
        elif len(recent) >= 1 and self.period is None and 20.0 < recent[-1] < 40.0:
            # Estimate quickly for display, but don't save as a stable period yet.
            self.period = recent[-1]
        if self.period is not None:
            self.predicted_next = event_t + self.period
        return event_t

    def is_fresh(self, now):
        return self.last_valid_t is not None and now - self.last_valid_t < 0.8


class CameraMonitor:
    def __init__(self, camera_index=0, calibration='vision_calibration.json',
                 tag_id=0, allow_recalibration=True, quiet=False):
        try:
            import cv2
        except ImportError as exc:
            raise RuntimeError('OpenCV unavailable: install opencv-contrib-python') from exc
        if not hasattr(cv2, 'aruco') or not hasattr(cv2.aruco, 'DICT_APRILTAG_36h11'):
            raise RuntimeError('OpenCV is missing AprilTag 36h11 support')
        self.cv2 = cv2
        self.camera_index = camera_index
        self.tag_id = tag_id
        self.calibration = Path(calibration)
        self.allow_recalibration = allow_recalibration
        self.quiet = quiet
        self.center = self.pickup = None
        self.cached_period = None
        self.saved_period = None
        self.saved_resolution = None
        if self.calibration.is_file():
            data = json.loads(self.calibration.read_text())
            self.center = tuple(data['center'])
            self.pickup = tuple(data['pickup'])
            saved = data.get('period_s')
            if saved is not None and 20.0 < float(saved) < 40.0:
                self.cached_period = float(saved)
                self.saved_period = self.cached_period
            self.saved_resolution = data.get('resolution')
            saved_camera = data.get('camera_index')
            if saved_camera is not None and int(saved_camera) != int(camera_index):
                if not allow_recalibration:
                    raise RuntimeError('Calibration belongs to another camera; run vision_timer')
                self.center = self.pickup = self.cached_period = self.saved_period = None
                self.saved_resolution = None
                print('Camera changed: click CENTER and PICK to recalibrate')
            print('Calibration loaded' +
                  (f' | period {self.cached_period:.3f}s' if self.cached_period else ' | learning period'))

        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            raise RuntimeError(f'Cannot open camera {camera_index}')
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.resolution = [int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                           int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))]
        if self.saved_resolution and list(self.saved_resolution) != self.resolution:
            if not allow_recalibration:
                self.cap.release()
                raise RuntimeError('Resolution changed; recalibrate using vision_timer')
            self.center = self.pickup = self.cached_period = self.saved_period = None
            print('Resolution changed: click CENTER and PICK to recalibrate')

        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        params = (cv2.aruco.DetectorParameters() if hasattr(cv2.aruco, 'ArucoDetector')
                  else cv2.aruco.DetectorParameters_create())
        # Include all manual on-lab improvements in the checked-in file.
        params.aprilTagQuadDecimate = 1.0
        params.aprilTagMinWhiteBlackDiff = 3
        params.minMarkerPerimeterRate = 0.01
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        if hasattr(cv2.aruco, 'ArucoDetector'):
            self.detector = cv2.aruco.ArucoDetector(dictionary, params)
            self.detect = self.detector.detectMarkers
        else:
            self.detect = lambda im: cv2.aruco.detectMarkers(im, dictionary, parameters=params)

        self.tracker = (RotationTracker(self.center, self.pickup, self.cached_period)
                        if self.center and self.pickup else None)
        self.window = 'AprilTag PICK timing'
        cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
        if self.allow_recalibration:
            cv2.setMouseCallback(self.window, self.on_mouse)
        self.flash_until = 0.0
        self.fired_prediction = None
        self.status = '' if allow_recalibration else 'WAITING'
        self.found = False
        if self.tracker is None and not allow_recalibration:
            self.close()
            raise RuntimeError('No saved calibration. Run vision_timer first.')
        if not quiet:
            print('Camera ready | r: reset  q: quit' if allow_recalibration
                  else 'Camera ready | calibration locked for robot run')

    def save(self, save_period=True):
        if self.center is None or self.pickup is None:
            return
        data = {'center': list(self.center), 'pickup': list(self.pickup),
                'camera_index': int(self.camera_index), 'resolution': self.resolution}
        if save_period and self.tracker and self.tracker.period:
            data['period_s'] = round(float(self.tracker.period), 6)
            self.saved_period = self.tracker.period
        self.calibration.write_text(json.dumps(data, indent=2) + '\n')

    def on_mouse(self, event, x, y, flags, param):
        if event != self.cv2.EVENT_LBUTTONDOWN:
            return
        if self.center is not None and self.pickup is not None:
            return  # Calibration locked until the user presses r.
        if self.center is None:
            self.center = (x, y)
            print('Center set | click PICK')
        else:
            self.pickup = (x, y)
            self.cached_period = self.saved_period = None
            self.tracker = RotationTracker(self.center, self.pickup)
            self.fired_prediction = None
            self.save(save_period=False)
            print('PICK set | saved')

    def reset(self):
        self.center = self.pickup = self.tracker = None
        self.cached_period = self.saved_period = None
        self.fired_prediction = None
        if self.calibration.exists():
            self.calibration.unlink()
        print('Reset | click CENTER, then PICK')

    def draw_text(self, frame, lines, y=28):
        cv2 = self.cv2
        for line in lines:
            cv2.putText(frame, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX,
                        0.67, (255, 255, 255), 3)
            cv2.putText(frame, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX,
                        0.67, (0, 0, 0), 1)
            y += 29

    def next_frame(self, lead=17.0, allow_quit=True):
        cv2 = self.cv2
        ok, frame = self.cap.read()
        if not ok:
            raise RuntimeError('Camera frame read failed')
        now = time.monotonic()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detect(gray)
        tag = None
        self.found = False
        if ids is not None:
            for quad, ident in zip(corners, ids.flatten()):
                if int(ident) == self.tag_id:
                    import numpy as np
                    pts = np.asarray(quad).reshape(4, 2)
                    tag = tuple(np.mean(pts, axis=0))
                    self.found = True
                    cv2.polylines(frame, [pts.astype('int32')], True, (0, 220, 0), 2)
                    cv2.circle(frame, tuple(map(int, tag)), 4, (0, 220, 0), -1)
                    break

        prediction = None
        crossing = None
        lines = [('TAG FOUND' if self.found else 'TAG NOT FOUND')]
        if self.center is not None and self.pickup is not None:
            center = tuple(map(int, self.center))
            pickup = tuple(map(int, self.pickup))
            radius = int(math.dist(center, pickup))
            cv2.circle(frame, center, max(1, radius), (255, 150, 30), 1)
            cv2.circle(frame, center, 5, (255, 150, 30), -1)
            cv2.circle(frame, pickup, 8, (0, 0, 255), 2)
            cv2.putText(frame, 'PICK', (pickup[0] + 10, pickup[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
            if tag is not None and self.tracker:
                crossing = self.tracker.update(now, tag)
                if (self.tracker.stable_period and self.tracker.period and
                        (self.saved_period is None or
                         abs(self.saved_period - self.tracker.period) > 0.06)):
                    self.save()
                    print(f'Period saved: {self.tracker.period:.3f}s')
            if self.tracker and self.tracker.period:
                period = self.tracker.period
                if self.allow_recalibration:
                    lines.append(f'Period {period:.3f}s | omega {TAU/period:.3f} rad/s')
                if self.tracker.crossings:
                    latest = self.tracker.crossings[-1]
                    if now - latest < period * 1.25:
                        prediction = latest + period
                        if prediction <= now:
                            prediction += period
                        lines.append(f'PICK in {prediction - now:.2f}s')
                    else:
                        lines.append('Waiting for PICK crossing')
                else:
                    lines.append('Waiting for PICK crossing')
                if self.tracker.direction is not None:
                    th_pick = angle_from(center, pickup)
                    th_start = th_pick - TAU / period * self.tracker.direction * lead
                    marker = (int(center[0] + radius * math.cos(th_start)),
                              int(center[1] - radius * math.sin(th_start)))
                    cv2.circle(frame, marker, 8, (0, 255, 255), 2)
                    cv2.putText(frame, 'START', (marker[0] + 10, marker[1] + 13),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            else:
                lines.append('Learning period')
        else:
            lines.append('Click CENTER' if self.center is None else 'Click PICK')

        if self.tracker and self.tracker.last_prediction_error is not None and self.allow_recalibration:
            lines.append(f'Last error {self.tracker.last_prediction_error:+.3f}s')
        if self.status:
            lines.append(self.status)

        if (self.allow_recalibration and prediction is not None and
                self.tracker.is_fresh(now) and 0 <= prediction - now <= lead and
                self.fired_prediction != prediction and
                abs(prediction - now - lead) <= 0.35):
            self.fired_prediction = prediction
            self.flash_until = now + 1.5
        if now < self.flash_until:
            cv2.rectangle(frame, (0, 0), (frame.shape[1], 53), (0, 110, 0), -1)
            self.draw_text(frame, ['START ARM NOW'], 36)
            self.draw_text(frame, lines, 84)
        else:
            self.draw_text(frame, lines)
        cv2.putText(frame,
                    ('r reset | q quit' if self.allow_recalibration
                     else 'RUN/CANCEL in terminal | camera open until DONE'),
                    (10, frame.shape[0]-13), cv2.FONT_HERSHEY_SIMPLEX,
                    0.53, (235, 235, 235), 1)
        cv2.imshow(self.window, frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('r') and self.allow_recalibration:
            self.reset()
        if key in (ord('q'), 27) and allow_quit:
            raise KeyboardInterrupt
        return now, crossing, prediction, self.found

    def close(self):
        if hasattr(self, 'cap'):
            self.cap.release()
        self.cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera', type=int, default=0)
    parser.add_argument('--lead', type=float, default=17.0)
    parser.add_argument('--calibration', default='vision_calibration.json')
    parser.add_argument('--tag-id', type=int, default=0)
    args = parser.parse_args()
    if args.lead <= 0:
        parser.error('--lead must be positive')
    monitor = CameraMonitor(args.camera, args.calibration, args.tag_id)
    try:
        while True:
            monitor.next_frame(args.lead)
    except KeyboardInterrupt:
        print('Camera closed')
    finally:
        monitor.close()


if __name__ == '__main__':
    main()
