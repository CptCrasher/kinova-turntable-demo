"""One-shot Kinova pick, timed from an Arduino Hall-sensor reference.

Starts in preview mode. Set execute:=true to send physical robot commands.
The Kinova custom service server (/home, /set_tool, /get_tool,
/set_gripper) must be running before enabling robot motion.
"""

import math
import time

import rclpy
from rclpy.node import Node
from kinova_gen3_interfaces.srv import Status, SetTool, GetTool, SetGripper


# Poses recorded from the Kinova controller: metres, degrees.
# Same orientation for vertical moves; check it on the real arm.
# MID is a NEW interpolated pose, not yet physically tested.
ORIENTATION = (-2.4, -177.9, 4.5)
ABOVE = (0.303, -0.012, 0.176)
MID = (0.303, -0.012, 0.112)
PICK = (0.303, -0.012, 0.049)
LIFT = (0.303, -0.012, 0.176)
EXIT = (0.319, -0.004, 0.175)

GRIP_OPEN = 0.0
GRIP_CLOSED = 0.92   # Test /set_gripper values before enabling this


class TurntablePick(Node):
    def __init__(self):
        super().__init__('turntable_pick')
        defaults = {
            'serial_port': '/dev/ttyACM0',
            'execute': False,
            'include_pick': False,
            'include_gripper': False,
            'simulate_hall': False,
            'sim_period_s': 30.0,
            # Seconds from Hall REF to block arriving at PICK.
            # Adjust this experimentally (block is opposite magnet).
            'pickup_delay_s': 8.0,
            # Motion begins this many seconds BEFORE block arrival.
            'approach_lead_s': 7.0,
            'mid_lead_s': 3.0,
            'pick_lead_s': 1.2,
            'grip_advance_s': 0.15,
            'grip_settle_s': 0.5,
            'max_late_s': 0.6,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)

        self.execute = self.p('execute')
        self.include_pick = self.p('include_pick')
        self.include_gripper = self.p('include_gripper')
        if self.include_gripper and not self.include_pick:
            raise ValueError('include_gripper requires include_pick:=true')
        if not (self.p('approach_lead_s') > self.p('mid_lead_s')
                > self.p('pick_lead_s') > self.p('grip_advance_s') >= 0):
            raise ValueError('Lead times must be approach > mid > pick > grip >= 0')

        self.board = None
        if self.execute:
            self.home = self.create_client(Status, '/home')
            self.tool = self.create_client(SetTool, '/set_tool')
            self.get_tool = self.create_client(GetTool, '/get_tool')
            self.gripper = self.create_client(SetGripper, '/set_gripper')

    def p(self, name):
        return self.get_parameter(name).value

    def call(self, client, request, label, timeout=60.0):
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f'{label}: service unavailable')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise RuntimeError(f'{label}: timed out; robot may still be moving')
        result = future.result()
        if result is None or (hasattr(result, 'status') and not result.status):
            raise RuntimeError(f'{label}: service failed')
        return result

    def move(self, label, xyz):
        print(f'{label}: {xyz} m')
        if not self.execute:
            return
        request = SetTool.Request()
        request.x, request.y, request.z = map(float, xyz)
        request.theta_x, request.theta_y, request.theta_z = ORIENTATION
        started = time.monotonic()
        self.call(self.tool, request, label)
        # The service may return before the tool actually reaches its target.
        deadline = time.monotonic() + 30
        consecutive = 0
        while time.monotonic() < deadline:
            actual = self.call(self.get_tool, GetTool.Request(), 'GET_TOOL', 5)
            error = math.dist(xyz, (actual.x, actual.y, actual.z))
            consecutive = consecutive + 1 if error < 0.010 else 0
            if consecutive >= 3:
                print(f'  reached (XYZ error {error * 1000:.1f} mm, '
                      f'{time.monotonic() - started:.2f} s)')
                return
            time.sleep(0.15)
        raise RuntimeError(f'{label}: target XYZ was not reached')

    def grip(self, label, value):
        print(f'{label}: gripper command {value}')
        if self.execute and self.include_gripper:
            request = SetGripper.Request()
            request.value = float(value)
            self.call(self.gripper, request, label)

    def wait_for(self, when, label):
        while time.monotonic() < when:
            time.sleep(min(0.02, max(0.0, when - time.monotonic())))
        late = time.monotonic() - when
        if late > self.p('max_late_s'):
            raise RuntimeError(f'Missed {label} by {late:.2f} s; no further commands')

    def get_references(self):
        if self.p('simulate_hall'):
            print('Simulated Hall reference')
            return time.monotonic(), float(self.p('sim_period_s'))

        try:
            import serial
        except ImportError as exc:
            raise RuntimeError('Install python3-serial') from exc

        port = self.p('serial_port')
        print(f'Opening Arduino {port}; waiting for two REF events...')
        # Opening serial resets many Uno/Nano boards. Allow it to reboot.
        # Leave USB serial OPEN throughout the pickup. Closing it early can
        # reset some Arduinos, which would disrupt turntable synchronization.
        self.board = serial.Serial(port, 115200, timeout=1)
        refs = []
        deadline = time.monotonic() + 150.0
        while len(refs) < 2 and time.monotonic() < deadline:
            line = self.board.readline().decode('ascii', errors='ignore').strip()
            if line == 'READY':
                print('Arduino ready')
            if not line.startswith('REF,'):
                continue
            try:
                stamp = int(line.split(',')[1])
            except (IndexError, ValueError):
                continue
            host_time = time.monotonic()
            if refs and ((stamp - refs[-1][1]) & 0xFFFFFFFF) < 5000000:
                continue  # Ignore unexpected duplicate hit
            refs.append((host_time, stamp))
            print(f'  Hall reference {len(refs)}/2')

        if len(refs) < 2:
            raise RuntimeError('No two Hall references received; check sensor/serial')
        period = ((refs[1][1] - refs[0][1]) & 0xFFFFFFFF) / 1_000_000.0
        if not 8 <= period <= 120:
            raise RuntimeError(f'Implausible measured period: {period:.2f} s')
        print(f'Measured revolution period: {period:.3f} s; RPM: {60 / period:.2f}')
        return refs[-1][0], period

    def run(self):
        print('\n=== TURNTABLE PICK ===')
        print('Robot commands:', 'ON' if self.execute else 'OFF (preview)')
        print('Pick descent:', self.include_pick,
              '| Gripper:', self.include_gripper)
        if self.execute:
            print('WARNING: this is timed open-loop interception, not live tracking.')
            reply = input('Clear path, motor/gripper checked, E-stop ready? Type RUN: ')
            if reply.strip() != 'RUN':
                print('Cancelled')
                return
            print('HOME')
            self.call(self.home, Status.Request(), 'HOME', 90)
            if self.include_gripper:
                self.grip('OPEN', GRIP_OPEN)

        ref_time, period = self.get_references()
        delay = float(self.p('pickup_delay_s'))
        if not 0 <= delay < period:
            raise ValueError(f'pickup_delay_s must be between 0 and {period:.2f} s')

        # If this pass is too close, target the next full revolution.
        hit_time = ref_time + delay
        while hit_time - self.p('approach_lead_s') < time.monotonic() + 0.75:
            hit_time += period
        print(f'Block expected at PICK in {hit_time - time.monotonic():.2f} s')

        self.wait_for(hit_time - self.p('approach_lead_s'), 'approach')
        self.move('ABOVE', ABOVE)

        if self.include_pick:
            self.wait_for(hit_time - self.p('mid_lead_s'), 'mid')
            self.move('MID', MID)
            self.wait_for(hit_time - self.p('pick_lead_s'), 'pick descent')
            self.move('PICK', PICK)
            self.wait_for(hit_time - self.p('grip_advance_s'), 'grip')
            if self.include_gripper:
                self.grip('CLOSE', GRIP_CLOSED)
            else:
                print('GRIP point reached (gripper disabled)')
            time.sleep(float(self.p('grip_settle_s')))
            self.move('LIFT', LIFT)

        else:
            self.wait_for(hit_time, 'planned pickup time')

        self.move('EXIT', EXIT)
        print('DONE: arm stays at EXIT; no DROP or return HOME')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TurntablePick()
        node.run()
    except (RuntimeError, ValueError, KeyboardInterrupt) as exc:
        print(f'STOPPED: {exc}')
        print('A timed-out ROS call does not stop the physical robot. Check the arm.')
    finally:
        if node is not None:
            if node.board is not None:
                node.board.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
