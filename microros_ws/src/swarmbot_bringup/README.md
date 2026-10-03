# swarmbot_bringup

Launch files that put SwarmBot into RViz, either purely simulated or driven by
the real ESP32-S3 over the micro-ROS agent.

## Naming convention (applies to every future robot_0N too)

- ROS namespace and TF `frame_prefix` are both set from the `robot_name`
  launch argument (default `robot_01`), matching the node name and topic
  prefix already hardcoded in the ESP32-S3 firmware (`robot_01/status`,
  `robot_01/led_cmd`, ...).
- TF frames therefore come out as `robot_01/base_link`,
  `robot_01/wheel_left_link`, etc. — this is the standard multi-robot-safe
  pattern (one global `/tf`, frames disambiguated by prefix) instead of a
  separate `/tf` topic per robot.
- Future sensor topics should keep following `/robot_0N/<topic>` (e.g.
  `/robot_01/odom`, `/robot_01/range_front`) — the RViz configs in
  `swarmbot_description/rviz/` already have displays wired to those names
  for `robot_01`, just disabled until data exists. `/robot_01/imu` is
  already live (real MPU6050, see below) — no Range/ToF data yet.

## `sim.launch.py` — RViz-only simulation, no hardware

```bash
ros2 launch swarmbot_bringup sim.launch.py
```

Starts `robot_state_publisher` + `joint_state_publisher_gui` (sliders to spin
the wheel joints) + RViz. No micro-ROS agent, no ESP32 required.

## `real_robot.launch.py` — real robot over Wi-Fi/UDP

```bash
ros2 launch swarmbot_bringup real_robot.launch.py
```

Starts `micro_ros_agent udp4 --port 8888` (same command as in
`~/swarmbot/firmware/README.md`), `robot_state_publisher`, `imu_orientation_tf`
(see below), a headless `joint_state_publisher` (holds the wheel joints at
rest until real encoder data exists), and RViz — using
`swarmbot_description/rviz/swarmbot_real.rviz` (not the same config as
`sim.launch.py`, see below).

Range/ToF firmware doesn't exist yet, so those RViz displays stay disabled
until that stage lands (enable them, or re-save the `.rviz` config with them
on by default, once `/robot_01/range_*` is actually being published).

### `imu_orientation_tf.py` — MPU6050 orientation preview (2026-09-04)

`scripts/imu_orientation_tf.py` subscribes to `/robot_01/imu` (published by
the firmware from a real MPU6050, see `~/swarmbot/microros_ws/README.md` section 6)
and broadcasts TF `robot_01/odom -> robot_01/base_link`. Deliberately:

- **Yaw only** — this is a differential-drive robot with a passive caster,
  so the chassis is always flat on the ground; roll/pitch are pinned to 0
  even though the MPU6050 reports them.
- **No translation** — always `(0, 0, 0)`. There's no encoder data yet, so
  there's nothing real to put there; this previews the future odometry's
  *orientation* only, not position.
- **Gyro Z bias auto-calibration** — averages the first ~1s of readings at
  startup (assumes the robot is still) and subtracts that offset from then
  on. Cuts yaw drift from ~47°/min (uncalibrated) to ~1.2°/min in testing —
  doesn't eliminate it (bias isn't perfectly constant, no magnetometer to
  anchor yaw absolutely).

Runs outside the `PushRosNamespace(robot_name)` group on purpose: the
firmware already bakes `robot_01/` into its topic names directly (not via
ROS namespacing), and this node must publish to the global `/tf` topic (not
a namespaced `/robot_01/tf`) for RViz and `robot_state_publisher` to see it
— same reasoning as the existing `frame_prefix` pattern.

**Fixed Frame:** `swarmbot_real.rviz` sets it to `robot_01/odom` (not
`robot_01/base_link` like `sim.launch.py`'s config), so the camera stays
anchored to a frame that doesn't rotate with the robot — that's how the
rotation becomes visible at all. `swarmbot.rviz` (sim) is untouched.

### Motor tuning/testing scripts (2026-09-12)

Not part of any launch file — run standalone against the real hardware,
with `micro_ros_agent` already connected:

- `scripts/tune_wheel_pi.py` — grid-searches Kp/Ki against the real motor
  (publishes to `<wheel>/kp`, `<wheel>/ki`, `<wheel>/target_velocity`,
  reads back `<wheel>/velocity`), scores by RMSE + overshoot penalty.
  `ros2 run swarmbot_bringup tune_wheel_pi.py --ros-args -p wheel:=wheel_left -p target:=2.0`
- `scripts/test_wheel_ramp.py` — ramps target 0→target_max→0 and reports
  tracking RMSE/overshoot over the whole trajectory (a step test alone can
  hide overshoot that only shows up at higher commanded speeds).
  `ros2 run swarmbot_bringup test_wheel_ramp.py --ros-args -p wheel:=wheel_right -p target_max:=3.0`

See `~/swarmbot/microros_ws/README.md` section 7 for the full story (pin
list, PI architecture, left-motor results, right-motor unresolved hardware
issue).

## Args

| Arg | Default | Used by |
|---|---|---|
| `robot_name` | `robot_01` | both |
| `agent_port` | `8888` | `real_robot.launch.py` |
