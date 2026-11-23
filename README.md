# Kinova + rotating turntable pickup

Simple one-shot demo. The Arduino spins the platform and prints `REF,<micros>` once per revolution. The ROS 2 node measures the revolution period from **two references** and schedules a preprogrammed Kinova motion. **No camera is required for robot control.**

**WARNING:** This is a timed open-loop demonstration, not safety-rated motion control or guaranteed moving-object grasping. Always test the arm with the table OFF and NO BLOCK first. Keep the E-stop accessible. The `MID` pose is an untested interpolation. The Kinova controller determines actual movement speed: splitting descent into two stages does not set a speed limit.

## Folder layout

```text
arduino/turntable_hall/turntable_hall.ino    Arduino firmware
ros2_ws/src/turntable_pick/                ROS 2 Python package
  package.xml
  setup.py
  setup.cfg
  resource/turntable_pick               (empty)
  turntable_pick/__init__.py             (empty)
  turntable_pick/pick.py                 Kinova + timing logic
```

## Hardware

- Arduino + 28BYJ-48 + ULN2003: IN1=D8, IN2=D9, IN3=D10, IN4=D11
- KY-024 Hall digital output D0 to Arduino D2, 5V and GND
- Magnet attached to the spinning disk; **block placed 180 degrees from magnet** on a repeatable mark
- External regulated 5V motor supply connected to ULN2003; connect its ground to Arduino ground
- KY-024 potentiometer adjusted for exactly one crossing per rotation
- Turntable base in a fixed, repeatable location relative to the arm

## Arduino

1. Open `arduino/turntable_hall/turntable_hall.ino` in Arduino IDE on ANY computer with Arduino IDE.
2. Install **AccelStepper** library; select the correct board and port; upload **once**.
3. Check the Serial Monitor at **115200 baud**: `READY` followed by one `REF,<micros>` per revolution.
4. Close Serial Monitor. Disconnect USB and connect Arduino by USB to the robot laptop. It will keep its uploaded firmware and restart automatically when powered; Arduino IDE is **not** required on the robot laptop. Keep motor power connected.
5. Do not unplug Arduino serial while the robot is executing the pickup.

If the magnet sensor produces multiple hits at its edge, the firmware filters flicker for 30 ms and ignores all new references for 8 seconds after a hit. **Only suitable for a revolution period well above 8 seconds.**

## ROS 2 installation (on robot laptop)

Use the existing Kinova workspace (e.g. `~/kinova/kinova_ws`) that provides custom services `/home`, `/set_tool`, `/get_tool`, `/set_gripper`. These are *course-specific* interfaces, not interchangeable with the official Kortex actions.

```bash
# From the clone root:
cd ~/turntable-kinova/ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/kinova/kinova_ws/install/setup.bash
sudo apt install python3-serial
colcon build --symlink-install
source install/setup.bash
ros2 pkg executables turntable_pick
```

Expected executable: `turntable_pick pick`. When opening a new terminal, source base ROS, Kinova install, then this repo's `ros2_ws/install/setup.bash` in that order. Rebuild after changing `setup.py` or `package.xml`; with `--symlink-install`, ordinary edits to `pick.py` generally need no rebuild.

Start the **existing** Kinova service server in terminal 1 using your known launch command. Test in terminal 2:

```bash
ros2 service list -t | grep -E 'home|tool|gripper'
ros2 service call /get_tool kinova_gen3_interfaces/srv/GetTool "{}"
```

**Do not enable physical commands unless the required Kinova services are available and the poses/paths have been verified.**

## Test sequence

All runs below are **one-shot** and finish after LIFT / EXIT. The robot does not drop or return HOME.

```bash
# A) Timing preview without Arduino or robot movements (about 8 seconds)
ros2 run turntable_pick pick --ros-args -p simulate_hall:=true

# B) Read actual Hall sensor, but print movement schedule only (NO robot movement)
ros2 run turntable_pick pick --ros-args -p serial_port:=/dev/ttyACM0

# C) With robot server running and table OFF: simulated trigger, upper movements only.
# Observe and approve newly taught paths before this test.
ros2 run turntable_pick pick --ros-args -p simulate_hall:=true -p execute:=true

# D) With table OFF and NO BLOCK, full descending movements, no gripper.
# Only after upper movement is safe and low PICK clearance is verified.
ros2 run turntable_pick pick --ros-args -p simulate_hall:=true -p execute:=true -p include_pick:=true

# E) With table OFF and a safe stationary block, test gripper only after
# verifying /set_gripper 0.0=open and 0.92=close on your robot.
ros2 run turntable_pick pick --ros-args -p simulate_hall:=true -p execute:=true -p include_pick:=true -p include_gripper:=true

# F) Only after all stationary tests: real Hall-timed run on rotating platform.
ros2 run turntable_pick pick --ros-args -p serial_port:=/dev/ttyACM0 -p pickup_delay_s:=8.0 -p execute:=true -p include_pick:=true -p include_gripper:=true
```

Every run with `execute:=true` requires typing `RUN` once at the beginning. Do **not** enable the full pickup until robot speed/timing and gripper values are validated. Stop with the physical robot E-stop if unsafe; killing a terminal or timing out a service does not stop motion already sent.

## Parameters to tune

| Parameter | Default | Meaning |
|---|---:|---|
| `pickup_delay_s` | `8.0` | Seconds between **magnet entering Hall sensor** and **block arriving at PICK**. Start with a measured/estimated value and tune. The block opposite the magnet alone does not establish this delay unless the pickup angle relative to the Hall sensor is known. |
| `approach_lead_s` | `7.0` | Start HOME -> ABOVE this far before arrival |
| `mid_lead_s` | `3.0` | Start ABOVE -> MID this far before arrival |
| `pick_lead_s` | `1.2` | Start MID -> PICK this far before arrival |
| `grip_advance_s` | `0.15` | Start closing this far before predicted arrival |
| `grip_settle_s` | `0.5` | Wait after gripper command before lifting |
| `max_late_s` | `0.6` | Abort if scheduled milestone is missed by more than this |

Example changing timing, retaining all other values:

```bash
ros2 run turntable_pick pick --ros-args -p serial_port:=/dev/ttyACM0 -p pickup_delay_s:=11.5
```

Robot poses in `ros2_ws/src/turntable_pick/turntable_pick/pick.py` are recorded XYZ, metres, and a shared orientation in degrees. `MID=(0.303,-0.012,0.112)` is new and **must be verified**. `PICK z=0.049` is low: note tool frame is not necessarily the bottom of the fingers. Robot service `/set_tool` provides position, **not tool speed**. Stage descent timing does not guarantee a low speed or successful moving grasp. Real-time tracking/velocity matching would be required for higher turntable speeds.

## If Arduino isn't found

Check `ls /dev/serial/by-id/` or `ls /dev/ttyACM* /dev/ttyUSB*` (board may appear as `/dev/ttyUSB0`). If Linux reports permission denied, check `groups` and ask a lab admin to add your user to `dialout` (may require logging out/in). Close Arduino Serial Monitor and other serial clients. `python3-serial` is needed on the robot laptop but Arduino IDE isn't.

## GitHub

Commit **source code only**. `.gitignore` excludes `ros2_ws/build`, `install`, and `log`. Clone the repo on the robot laptop and build there. The Kinova workspace remains separate; don't copy `kinova_gen3_interfaces` into this repository.
