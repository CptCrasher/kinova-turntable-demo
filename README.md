# Kinova Gen3 — Rotating Turntable Pickup

A Kinova Gen3 picks up a block from a **continuously rotating turntable**. There are **two independent ways to time the same recorded robot movements**:

1. **AprilTag camera (main version):** an overhead camera sees the tag on the block, measures full revolutions, and predicts when the tag will reach PICK.
2. **Arduino Hall sensor (backup version):** a magnet triggers a KY-024 sensor once per revolution; the program uses a timed offset to predict PICK.

**These are one-shot, timed pickups—not real-time visual servoing or collision avoidance.** The arm moves to fixed poses, closes the gripper, lifts, and stays at EXIT. It does not return HOME or drop the block. Always verify clearance and have the physical E-stop accessible. A ROS timeout or Ctrl+C is **not** a robot emergency stop.

---

# Part 1 — AprilTag camera version (recommended)

## 1. Set up after cloning on `rob05`

This project uses **ROS 2 Humble** and the lab's existing Kinova ROS 2 services. The Kinova driver/service server is in a **separate workspace**: `~/kinova/kinova_ws`.

```bash
cd ~/kinova-turntable-demo/ros2_ws

# Source the lab ROS installation and the existing Kinova interfaces
source /opt/ros/humble/setup.bash
source ~/kinova/kinova_ws/install/setup.bash

# Build this project
colcon build --symlink-install --packages-select turntable_pick
source install/setup.bash

# Confirm the three commands exist
ros2 pkg executables turntable_pick
```

Expected console entries (after the vision add-on has been committed to the repo):

```text
turntable_pick pick           # Original Hall-sensor version
turntable_pick vision_timer   # Camera only, no arm motion
turntable_pick vision_pick    # Camera-timed Kinova movements
```

**If `vision_timer` or `vision_pick` is missing:** make sure `vision_camera.py`, `vision_pick.py`, and the corresponding entries in `ros2_ws/src/turntable_pick/setup.py` were committed and pulled; then rebuild. The AprilTag add-on does **not** replace `pick.py`.

### Python and camera dependencies

On the computer **with the camera attached**, check:

```bash
python3 -c "import cv2, numpy; print('OpenCV:', cv2.__version__, 'AprilTags:', hasattr(cv2, 'aruco'))"
```

If those modules are unavailable, install `opencv-contrib-python` and `numpy` for the **same Python** that runs ROS 2, using your lab's approved install method. For example, if user-site pip installs are permitted:

```bash
python3 -m pip install --user opencv-contrib-python numpy
```

Avoid mixing incompatible versions of system OpenCV and pip OpenCV. Verify `python3 -c "import cv2; print(hasattr(cv2.aruco, 'DICT_APRILTAG_36h11'))"` afterward. ROS 2 and OpenCV must be importable in the same environment; a plain virtual environment may hide the system ROS packages.

**Important:** The overhead USB camera must be accessible from the machine running `vision_timer` / `vision_pick` (ideally `rob05`). If connected only to a personal laptop, `rob05` cannot automatically access it. An SSH-only session may also lack a graphical display for the OpenCV camera window.

For **each new terminal** after building, source in this order:

```bash
source /opt/ros/humble/setup.bash
source ~/kinova/kinova_ws/install/setup.bash
source ~/kinova-turntable-demo/ros2_ws/install/setup.bash
```

## 2. Prepare the physical setup

1. Mount the camera **above the turntable**, pointed roughly straight down. Keep it fixed throughout the run.
2. Print **AprilTag family 36h11, ID 0**, and attach it flat to the block's top surface. The project used a **23 mm × 23 mm** tag.
3. Ensure the whole path of the tag is visible, in focus, and well lit.
4. Put the turntable in its **repeatable, calibrated position** under the Kinova. Do not guess that the recorded PICK pose matches a newly moved turntable.
5. Start the turntable spinning. Its observed period has been approximately **30.1 seconds/revolution** (~1.99 RPM).

## 3. Test the camera without moving the robot

```bash
cd ~/kinova-turntable-demo/ros2_ws
ros2 run turntable_pick vision_timer --camera 0 --lead 17
```

A camera window opens. On the first run:

1. **Click the center of the turntable.**
2. **Click the PICK location**, where the **center of the AprilTag** should be when the block reaches the robot's grasp position. Click on the tag's circular orbit, not the disk's outside edge.
3. The program saves the two clicks to `vision_calibration.json` in the **current working directory**.
4. Keep the turntable spinning. Wait for at least **two observed PICK crossings** for a revolution-period estimate; watch several more to check the prediction error.
5. Press **`r`** to recalibrate or **`q`** to quit.

Watch the window/terminal for:

```text
PICK CROSSING ...
Measured full-revolution period: ~30.1 s
>>> START ARM NOW ...
Previous prediction error: ... s
```

**What does `--lead 17` mean?** It tells the *camera-only viewer* to print **START ARM NOW 17 seconds before its predicted PICK crossing**. It does **not** change the block speed, improve camera accuracy, or control the Kinova. In `vision_pick`, the equivalent movement parameter is `approach_lead_s` (see below).

Look for stable measured revolutions and prediction errors close to zero. If errors are consistently large, **fix the camera tracking/calibration**, not the `--lead` number. Changing the camera position or resolution invalidates the saved pixel coordinates; press `r` and recalibrate. Do not blindly reuse your personal-laptop `vision_calibration.json` on `rob05`.

## 4. Start the lab's Kinova ROS service server

In a **separate terminal** on `rob05`, source the lab Kinova workspace and launch the course's normal Kinova service server (the exact executable depends on the lab setup):

```bash
source /opt/ros/humble/setup.bash
source ~/kinova/kinova_ws/install/setup.bash
ros2 pkg executables kinova_gen3
# Start the correct server executable from the list, for example:
# ros2 run kinova_gen3 <your-server-executable>
```

In your turntable terminal, verify the services:

```bash
ros2 service list -t | grep -E '/home|/set_tool|/get_tool|/set_gripper|/get_gripper'
ros2 service call /get_tool kinova_gen3_interfaces/srv/GetTool '{}'
```

The controller uses the **course's custom `kinova_gen3_interfaces` services**. Do not substitute a generic Kortex service command.

## 5. Run the camera-timed robot sequence

**A. First: preview (NO robot commands)**

```bash
ros2 run turntable_pick vision_pick --ros-args \
  -p camera_index:=0 \
  -p approach_lead_s:=17.0 \
  -p mid_lead_s:=8.0 \
  -p pick_lead_s:=6.5 \
  -p grip_advance_s:=4.5 \
  -p grip_settle_s:=3.0 \
  -p include_pick:=true \
  -p include_gripper:=true
```

The program observes **at least 3 camera PICK crossings**, measures the period, and schedules one pickup. Because `execute` defaults to `false`, the printed poses and gripper commands are **only a preview**. `vision_pick` uses the camera's PICK crossing directly; **`pickup_delay_s` is Hall-only and does not shift camera timing**.

**B. Next: upper-arm-only motion (real robot; no descent or grip)**

With the block removed, use a **flat, low-profile AprilTag marker on the turntable at the same radius** so the camera can still track it. Confirm that the moving turntable clears the upper robot trajectory.

```bash
ros2 run turntable_pick vision_pick --ros-args \
  -p camera_index:=0 \
  -p execute:=true \
  -p include_pick:=false \
  -p include_gripper:=false \
  -p approach_lead_s:=17.0 \
  -p mid_lead_s:=8.0 \
  -p pick_lead_s:=6.5 \
  -p grip_advance_s:=4.5
```

Type `RUN` at the confirmation prompt. It homes, moves to ABOVE, waits for the scheduled pickup time, then goes to EXIT. **It will not descend or close the gripper.**

**C. Full motion / gripping: only after clearance and gripping tests**

After verifying stationary grasps and an empty-gripper descent with a clear turntable, the full motion command is the preview command above with **`-p execute:=true`** added. This enables HOME → OPEN → ABOVE → MID → PICK → CLOSE → LIFT → EXIT.

**CAUTION:** The previously used `grip_settle_s:=3.0` lets the fingers finish closing in stationary tests, but the block keeps rotating during that wait. **Do not assume a 3-second hold at fixed PICK is safe for a spinning-block grasp**: the block can be dragged or jammed. Establish a safe grasp-to-lift time and clearance before attempting a live pickup. The camera stops tracking once it has provided the one-shot reference; losing the block later does not automatically halt the arm.

## 6. Robot positions and movement timing

These values are **hardcoded in** `ros2_ws/src/turntable_pick/turntable_pick/pick.py` and reused by `vision_pick.py`. XYZ positions are in **metres**; orientation is in **degrees**.

| Position | X | Y | Z | Purpose |
|---|---:|---:|---:|---|
| ABOVE | 0.303 | -0.012 | 0.176 | Approach above the block |
| MID | 0.303 | -0.012 | 0.112 | Intermediate descent waypoint |
| PICK | 0.303 | -0.012 | 0.049 | Low grasp position |
| LIFT | 0.303 | -0.012 | 0.176 | Raise the held block |
| EXIT | 0.319 | -0.004 | 0.175 | Move out of pickup area |

Shared tool orientation: **`theta_x=-2.4°`, `theta_y=-177.9°`, `theta_z=4.5°`**. The HOME pose is commanded by the lab's `/home` service (not stored as an XYZ constant here).

Measured movement durations from lab tests (approximate, not guarantees):

| Movement | Observed duration |
|---|---:|
| HOME → ABOVE | 7.6 s |
| ABOVE → MID | 1.27 s |
| MID → PICK | 1.25 s |
| PICK → LIFT | 1.60 s |
| LIFT → EXIT | 0.9 s |

Gripper positions in `pick.py`: `GRIP_OPEN = 0.0`, `GRIP_CLOSED = 0.92`. The gripper position **is not currently a ROS parameter**. `grip_settle_s` adds waiting time after the CLOSE service response; it does **not** increase finger speed or change how far the fingers close.

### Camera-pick parameters

| Parameter | Example | What it does |
|---|---:|---|
| `camera_index` | `0` | Which USB camera OpenCV opens |
| `vision_calibration` | `vision_calibration.json` | File with center and PICK clicks; path relative to where you launch unless absolute |
| `vision_tag_id` | `0` | AprilTag ID to track (36h11 family) |
| `vision_min_crossings` | `3` | Minimum observed crossings before a one-shot reference |
| `execute` | `false` | `false` preview; `true` physical robot commands |
| `include_pick` | `false` | Whether to descend through MID and PICK and then LIFT |
| `include_gripper` | `false` | Whether to OPEN/CLOSE fingers; requires `include_pick:=true` |
| `approach_lead_s` | `17.0` | Start ABOVE movement this many seconds **before PICK** |
| `mid_lead_s` | `8.0` | Begin MID movement this many seconds before PICK |
| `pick_lead_s` | `6.5` | Begin final descent this many seconds before PICK |
| `grip_advance_s` | `4.5` | Send CLOSE this many seconds before PICK, **after reaching PICK** |
| `grip_settle_s` | `3.0` | Wait after CLOSE service response before LIFT |
| `max_late_s` | `0.6` | Abort further scheduled commands if a milestone is too late (not an E-stop) |

Required ordering: `approach_lead_s > mid_lead_s > pick_lead_s > grip_advance_s >= 0`. The timings above are the **most recent experimental settings**, not values guaranteed to make a centered grasp.

**Important timing detail:** The code waits for each `move()` to reach its target before scheduling the next step. If the robot reaches PICK late, increasing `grip_advance_s` by itself **cannot** make CLOSE earlier; you must also allow enough time to reach ABOVE/MID/PICK. The camera-timer `--lead` option and the `vision_pick` ROS motion lead parameters are **not automatically linked**.

---
---

# Part 2 — Arduino Hall-effect sensor version (backup)

This version uses the **same Kinova poses and movement code**, but gets its timing reference from an **Arduino + KY-024 Hall sensor** instead of the camera.

## 1. How the Hall version works

1. A 28BYJ-48 stepper motor spins the 3D-printed turntable continuously through a ULN2003 driver.
2. A magnet attached under the disk passes the KY-024 sensor once each revolution.
3. Arduino prints `READY` on startup and `REF,<microseconds>` once per detected revolution over **115200-baud USB serial**.
4. `ros2 run turntable_pick pick` reads **two REF messages**, calculates the revolution period (typically **30.13–30.15 s**), and predicts when the block will reach PICK using `pickup_delay_s`.
5. The robot executes the same ABOVE → MID → PICK → CLOSE → LIFT → EXIT sequence.

The block is mounted approximately **180° opposite the magnet**. If PICK is at the Hall sensor's angular position, half a revolution is approximately **15.07 seconds**. In earlier physical tuning, **`pickup_delay_s:=12.5`** was used to shift the arm schedule; treat this as an **experimental setting**, not a verified geometric offset. Changing it moves the whole plan relative to the Hall reference.

### Hardware / wiring

| Connection | Wiring |
|---|---|
| ULN2003 IN1/IN2/IN3/IN4 | Arduino D8 / D9 / D10 / D11 |
| KY-024 D0 (digital output) | Arduino D2 |
| KY-024 power | Arduino 5V and GND |
| Stepper | Plug 28BYJ-48 into ULN2003 board |
| Motor power | Regulated external 5V supply to ULN2003; **common ground** with Arduino |

Arduino sketch: `arduino/turntable_hall/turntable_hall.ino`. Upload once using Arduino IDE with the **AccelStepper** library installed. The sketch uses **115200 baud**. The robot laptop does not need Arduino IDE once the sketch is uploaded.

## 2. Get Arduino USB serial access on `rob05`

Connect the Arduino over USB to `rob05` and run:

```bash
ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
ls -l /dev/ttyACM0
groups
```

Usually the port is `/dev/ttyACM0`, but it can change. Your Linux login needs access to the serial port, typically via the **`dialout`** group. If `dialout` is not listed in `groups`, ask a lab administrator to add you. The admin can run:

```bash
sudo usermod -aG dialout "$USER"
```

Log **out and back in** (or start a new group-aware login session) before checking `groups` again. Group changes do not normally take effect in the existing terminal. Do not try to bypass lab-managed permissions.

**Check the serial stream without needing pyserial:**

```bash
stty -F /dev/ttyACM0 115200 raw -echo
cat /dev/ttyACM0
```

Expected output:

```text
READY
REF,564276
REF,30700608
REF,60839988
...
```

The REF lines should be approximately **30.1 seconds apart**. `READY` can appear again when opening the serial port resets the Arduino. Press **Ctrl+C** to quit `cat` before starting the ROS node, because only one application should read the serial stream at once.

### Python serial dependency

The ROS Hall program imports **`serial` from pyserial** (this is separate from the `stty`/`cat` test). Verify:

```bash
python3 -c "import serial; print(serial.__version__)"
```

If missing and a user install is permitted:

```bash
python3 -m pip install --user pyserial
```

If user-site installs are unavailable but pip works, an alternative without sudo is:

```bash
python3 -m pip install --target "$HOME/ros_python_deps" pyserial
export PYTHONPATH="$HOME/ros_python_deps:$PYTHONPATH"
```

Repeat the `export PYTHONPATH=...` in any new terminal using that target directory.

## 3. Run the Hall version

Source the three ROS workspaces as in Part 1 and start the **lab Kinova service server** separately for physical motion.

**A. Read Arduino timing only (safe preview):**

```bash
ros2 run turntable_pick pick --ros-args \
  -p serial_port:=/dev/ttyACM0 \
  -p pickup_delay_s:=12.5 \
  -p approach_lead_s:=17.0 \
  -p mid_lead_s:=8.0 \
  -p pick_lead_s:=6.5 \
  -p grip_advance_s:=4.5 \
  -p grip_settle_s:=3.0 \
  -p include_pick:=true \
  -p include_gripper:=true
```

The arm **will not move**: `execute` defaults to `false`. Watch for `Hall reference 1/2`, `Hall reference 2/2`, and the calculated revolution period.

**B. Real Hall-triggered upper-arm-only test:**

With the block removed, and **verified clearance** around the spinning turntable:

```bash
ros2 run turntable_pick pick --ros-args \
  -p serial_port:=/dev/ttyACM0 \
  -p pickup_delay_s:=12.5 \
  -p execute:=true \
  -p include_pick:=false \
  -p include_gripper:=false \
  -p approach_lead_s:=17.0 \
  -p mid_lead_s:=8.0 \
  -p pick_lead_s:=6.5 \
  -p grip_advance_s:=4.5
```

**C. Full Hall pickup:** Only after confirming stationary grasp, gripper closure, timing, and collision-free descent, use the preview command in A with **`-p execute:=true`** added. `include_pick:=true` enables descent; `include_gripper:=true` enables OPEN/CLOSE. Do not automatically reuse the 3-second settle delay for a moving block: a fixed gripper can drag or jam the rotating object.

### Why does it say `Block expected at PICK in 42.6 s`?

The program requires time to begin its approach. With `pickup_delay_s:=12.5` and `approach_lead_s:=17.0`, it **cannot** start 17 seconds before an arrival only 12.5 seconds after the latest Hall event. It therefore targets **the next revolution**, adding about **30.1 seconds**: `12.5 + 30.1 ≈ 42.6 s`. **This is expected and is not a slower turntable.**

## 4. Hall parameters: what to change

| Parameter | Example | What it controls |
|---|---:|---|
| `serial_port` | `/dev/ttyACM0` | Arduino USB device |
| `pickup_delay_s` | `12.5` | Predicted time from Hall REF to block arrival; shifts **all** robot events |
| `approach_lead_s` | `17.0` | Start ABOVE movement this far before predicted arrival |
| `mid_lead_s` | `8.0` | Start MID movement this far before predicted arrival |
| `pick_lead_s` | `6.5` | Start descent to PICK this far before predicted arrival |
| `grip_advance_s` | `4.5` | Start gripper CLOSE this far before predicted arrival, **if PICK has finished** |
| `grip_settle_s` | `3.0` | Wait after CLOSE service response, **before lifting** |
| `execute` | `false` | `true` actually commands the arm |
| `include_pick` | `false` | Whether to descend to PICK and then LIFT |
| `include_gripper` | `false` | Whether to OPEN and CLOSE the gripper |
| `max_late_s` | `0.6` | Stop issuing later steps if too late (not an E-stop) |

**For earlier CLOSE:** increase `grip_advance_s`, **and** schedule ABOVE/MID/PICK early enough that the arm is already at PICK. **For later CLOSE:** decrease `grip_advance_s`. **For more time to finish closing before LIFT:** increase `grip_settle_s` (but this keeps the arm at PICK longer and can be hazardous with a moving block). **For the entire sequence earlier/later relative to Hall:** decrease/increase `pickup_delay_s` *only if the observed arrival phase supports it*.

The script uses hardcoded `GRIP_CLOSED = 0.92`; changing `grip_settle_s` does not change the commanded finger position. Check actual closure with:

```bash
ros2 service call /get_gripper kinova_gen3_interfaces/srv/GetGripper '{}'
```

## 5. Hall fallback modes / common problems

**Manual reference (if Arduino serial isn't accessible):**

```bash
ros2 run turntable_pick pick --ros-args \
  -p manual_hall:=true \
  -p manual_period_s:=30.14 \
  -p pickup_delay_s:=12.5
```

The program asks you to press Enter as the magnet passes the sensor mark. This is less precise because of human reaction time. Leave `execute` at its default `false` for the first test. An optional `udp_hall` mode also exists for forwarding real REF messages from another machine, but requires network access and a separate forwarding script.

**Troubleshooting:**

- `Permission denied` on `/dev/ttyACM0`: check `groups`, `ls -l`, admin-provided `dialout`, and fresh login.
- `ModuleNotFoundError: serial`: install **pyserial** for the Python environment running ROS.
- `No two Hall references received`: inspect the Arduino at 115200 baud; check magnet position, sensor sensitivity, port and power.
- Camera can't open: check `/dev/video*`, webcam connection, camera index and GUI/display availability.
- Vision prediction error is high: improve lighting and focus; recalibrate center/PICK; ensure the tag stays visible; compare **measured full-revolution** periods rather than adjusting `--lead` blindly.
- Robot stops at `Missed mid` / `Missed pick`: a previous move took longer than the available lead interval. Re-measure movement durations; schedule it earlier instead of increasing `max_late_s` blindly.
- `STOPPED` or Ctrl+C: **does not guarantee that already-commanded physical motion has stopped**. Use the robot's actual safety controls.

---

## Git: update the repository README

From your **Windows PowerShell** project folder after placing this file at the repository root:

```powershell
cd X:\EECS4421\PROJECTDEMO\kinova-turntable-demo
git add README.md
git commit -m "Rewrite setup guide for vision and Hall pickup"
git push
```

On `rob05`, retrieve the updated guide and source files with `cd ~/kinova-turntable-demo && git pull`.
