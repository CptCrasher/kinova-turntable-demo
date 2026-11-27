#!/usr/bin/env python3
"""Camera-clocked Kinova PICK. Hall-based `pick.py` remains unchanged.

    ros2 run turntable_pick vision_pick --ros-args \
      -p execute:=false -p camera_index:=0 \
      -p approach_lead_s:=17.0 -p mid_lead_s:=8.0 \
      -p pick_lead_s:=6.5 -p grip_advance_s:=4.5

Camera is used as a clock to schedule ONE preprogrammed pick, NOT to track
or follow the moving block in real time. Fully physical runs require pose
verification, adequate clearance, and supervised E-stop readiness.
"""
import rclpy
from .pick import TurntablePick
from .vision_camera import CameraMonitor


class VisionPick(TurntablePick):
    def __init__(self):
        super().__init__()
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('vision_calibration', 'vision_calibration.json')
        self.declare_parameter('vision_tag_id', 0)
        self.declare_parameter('vision_min_crossings', 3)

    def get_references(self):
        # Keep existing TurntablePick's scheduling and motion *unchanged*.
        # It expects `reference_time + pickup_delay_s` to be a PICK time.
        # The camera measures PICK crossings directly, so cancel that Hall-only
        # offset. The public `vision_pick` timing is not influenced by it.
        monitor = CameraMonitor(camera_index=int(self.p('camera_index')),
                                calibration=str(self.p('vision_calibration')),
                                tag_id=int(self.p('vision_tag_id')))
        try:
            crossing, period = monitor.get_robust_reference(
                min_crossings=max(3, int(self.p('vision_min_crossings'))))
        finally:
            monitor.close()
        print('Camera predicts next PICK; executing normal Kinova schedule.')
        print('NOTE: vision is a one-shot timing reference, not live tracking.')
        return crossing - float(self.p('pickup_delay_s')), period


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VisionPick()
        node.run()
    except (RuntimeError, ValueError, KeyboardInterrupt) as exc:
        print(f'STOPPED: {exc}')
        print('A timed-out ROS call does NOT stop the physical robot; check the arm.')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
