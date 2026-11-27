#!/usr/bin/env python3
"""Camera-timed Kinova pickup. Original Hall `pick` executable is unchanged.

The camera stays open throughout execution. The operator types RUN to arm
one pickup, then CONTINUE to release at EXIT and return HOME. Camera timing
only establishes a one-shot predicted PICK timestamp; it is not visual servoing.
"""
import math
import threading
import time

import rclpy
from kinova_gen3_interfaces.srv import Status, SetTool, GetTool, SetGripper, GetGripper

from .pick import (TurntablePick, ABOVE, MID, PICK, LIFT, EXIT,
                   ORIENTATION, GRIP_OPEN, GRIP_CLOSED)
from .vision_camera import CameraMonitor


class VisionClock:
    """Handoff from the main OpenCV/UI thread to the ROS worker thread."""
    def __init__(self):
        self.condition = threading.Condition()
        self.crossing = None
        self.period = None
        self.fresh = False
        self.stopped = False

    def publish(self, monitor, now):
        tracker = monitor.tracker
        with self.condition:
            self.fresh = bool(tracker and monitor.found and tracker.is_fresh(now))
            if tracker and tracker.period and tracker.crossings:
                self.crossing = tracker.crossings[-1]
                self.period = tracker.period
            elif tracker is None:
                self.crossing = self.period = None
            self.condition.notify_all()

    def stop(self):
        with self.condition:
            self.stopped = True
            self.condition.notify_all()

    def wait_for_phase(self, timeout=150.0):
        """Require a current observed crossing, not only a stored period."""
        deadline = time.monotonic() + timeout
        with self.condition:
            while not self.stopped:
                now = time.monotonic()
                if (self.fresh and self.period and self.crossing and
                        20.0 < self.period < 40.0 and
                        0 <= now - self.crossing < self.period * 1.15):
                    return self.crossing, self.period
                remaining = deadline - now
                if remaining <= 0:
                    raise RuntimeError('No fresh AprilTag PICK crossing')
                self.condition.wait(timeout=min(0.2, remaining))
        raise RuntimeError('Camera stopped')


class VisionPick(TurntablePick):
    def __init__(self):
        super().__init__()
        self.declare_parameter('camera_index', 4)
        self.declare_parameter('vision_calibration', 'vision_calibration.json')
        self.declare_parameter('vision_tag_id', 0)
        self.clock = VisionClock()
        self.done = threading.Event()
        self.busy = threading.Event()
        self.status = 'HOMING' if self.execute else 'READY'
        self.error = None
        self._validate_timing()
        if self.execute and self.include_gripper:
            self.get_gripper_client = self.create_client(GetGripper, '/get_gripper')

    def _validate_timing(self):
        if not (self.p('approach_lead_s') > self.p('mid_lead_s') >
                self.p('pick_lead_s') > self.p('grip_advance_s') >= 0):
            raise ValueError('Expected approach > mid > pick > grip >= 0')
        if self.p('grip_settle_s') < 0:
            raise ValueError('grip_settle_s must be nonnegative')

    def move(self, label, xyz):
        self.status = label
        if not self.execute:
            print(f'{label} (preview)', flush=True)
            return
        request = SetTool.Request()
        request.x, request.y, request.z = map(float, xyz)
        request.theta_x, request.theta_y, request.theta_z = ORIENTATION
        started = time.monotonic()
        self.call(self.tool, request, label)
        deadline = time.monotonic() + 30.0
        consecutive = 0
        while time.monotonic() < deadline:
            actual = self.call(self.get_tool, GetTool.Request(), 'GET_TOOL', 5)
            error = math.dist(xyz, (actual.x, actual.y, actual.z))
            consecutive = consecutive + 1 if error < 0.010 else 0
            if consecutive >= 3:
                print(f'{label} reached | {time.monotonic()-started:.2f}s | '
                      f'{error*1000:.1f}mm', flush=True)
                return
            time.sleep(0.15)
        raise RuntimeError(f'{label}: position not reached')

    def grip(self, label, value):
        self.status = label
        print(f'{label} gripper {value:.2f}', flush=True)
        if self.execute and self.include_gripper:
            req = SetGripper.Request()
            req.value = float(value)
            self.call(self.gripper, req, label)

    def _home(self):
        self.status = 'HOME'
        if self.execute:
            self.call(self.home, Status.Request(), 'HOME', 90)
        print('HOME', flush=True)

    def _wait_gripper_open(self):
        """Do not HOME until the gripper has actually opened at EXIT."""
        deadline = time.monotonic() + 8.0
        stable = 0
        while time.monotonic() < deadline:
            result = self.call(self.get_gripper_client, GetGripper.Request(),
                               'GET_GRIPPER', 5.0)
            stable = stable + 1 if result.value < 0.06 else 0
            if stable >= 2:
                return
            time.sleep(0.15)
        raise RuntimeError('Gripper did not fully open; remaining at EXIT')

    def _prompt(self, message, word):
        while not self.clock.stopped:
            try:
                reply = input(message).strip().upper()
            except EOFError:
                raise RuntimeError('Terminal input unavailable')
            if reply == word:
                return
            if reply in ('CANCEL', 'QUIT', 'Q'):
                raise RuntimeError('Cancelled')
            print(f'Type {word} or CANCEL.', flush=True)
        raise RuntimeError('Stopped')

    def _schedule(self, reference, period):
        lead = float(self.p('approach_lead_s'))
        hit_time = reference + period
        while hit_time - lead < time.monotonic() + 0.75:
            hit_time += period
        return hit_time

    def run_once(self):
        print('VISION PICK | ' + ('ROBOT ON' if self.execute else 'PREVIEW'), flush=True)
        self._home()
        if self.include_gripper:
            self.grip('OPEN', GRIP_OPEN)
        self.status = 'READY - TYPE RUN'
        self._prompt('Type RUN to pick next available revolution: ', 'RUN')
        self.busy.set()
        self.status = 'WAITING FOR TAG'
        print('Arming from current camera crossing...', flush=True)
        crossing, period = self.clock.wait_for_phase()
        hit_time = self._schedule(crossing, period)
        print(f'PICK scheduled in {hit_time - time.monotonic():.1f}s '
              f'| period {period:.3f}s', flush=True)
        self.status = 'WAITING FOR APPROACH'
        self.wait_for(hit_time - self.p('approach_lead_s'), 'approach')
        self.move('ABOVE', ABOVE)

        if self.include_pick:
            self.status = 'WAITING FOR MID'
            self.wait_for(hit_time - self.p('mid_lead_s'), 'mid')
            self.move('MID', MID)
            self.status = 'WAITING FOR PICK'
            self.wait_for(hit_time - self.p('pick_lead_s'), 'pick descent')
            self.move('PICK', PICK)
            self.status = 'WAITING FOR CLOSE'
            self.wait_for(hit_time - self.p('grip_advance_s'), 'grip')
            if self.include_gripper:
                self.grip('CLOSE', GRIP_CLOSED)
            else:
                print('GRIP point (disabled)', flush=True)
            time.sleep(float(self.p('grip_settle_s')))
            self.move('LIFT', LIFT)
        else:
            self.status = 'WAITING FOR PICK TIME'
            self.wait_for(hit_time, 'pickup time')

        self.move('EXIT', EXIT)
        self.busy.clear()
        self.status = 'AT EXIT - TYPE CONTINUE'
        if self.execute:
            self._prompt('At EXIT: support the block, then type CONTINUE to release + HOME: ',
                         'CONTINUE')
            self.busy.set()
            if self.include_gripper:
                self.grip('OPEN', GRIP_OPEN)
                self._wait_gripper_open()
            self._home()
        self.status = 'DONE'
        print('DONE', flush=True)

    def run_worker(self):
        try:
            self.run_once()
        except (RuntimeError, ValueError, KeyboardInterrupt) as exc:
            self.error = exc
            self.status = f'STOPPED: {str(exc)[:65]}'
            print(f'STOPPED: {exc}', flush=True)
        finally:
            self.busy.clear()
            self.done.set()


def main(args=None):
    rclpy.init(args=args)
    node = None
    monitor = None
    try:
        node = VisionPick()
        monitor = CameraMonitor(camera_index=int(node.p('camera_index')),
                                calibration=str(node.p('vision_calibration')),
                                tag_id=int(node.p('vision_tag_id')),
                                allow_recalibration=False,
                                quiet=True)
        worker = threading.Thread(target=node.run_worker, name='kinova-pick', daemon=True)
        worker.start()
        while not node.done.is_set():
            monitor.status = node.status
            now, _, _, _ = monitor.next_frame(
                lead=float(node.p('approach_lead_s')),
                allow_quit=False)
            node.clock.publish(monitor, now)
        # A final camera frame keeps the window up until execution is actually done.
    except KeyboardInterrupt:
        print('Camera closed', flush=True)
    except (RuntimeError, ValueError) as exc:
        print(f'STOPPED: {exc}', flush=True)
    finally:
        if node is not None:
            node.clock.stop()
        if monitor is not None:
            monitor.close()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
