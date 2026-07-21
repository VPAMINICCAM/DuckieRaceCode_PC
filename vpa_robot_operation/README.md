# VPA_ROBOT_OPERATION

This package allows operating the robots in MiniCCAM, VPA for a complete process.

It calls for the lower layer packages, including robot interface for traction and sensors, robot perception for preprocessing the data from the sensors.

## DuckieRace

Deploy the same repository revision to every race robot, set its
`robot_name` environment variable, and launch:

```bash
roslaunch vpa_robot_operation duckierace_start.launch
```

The main-PC virtual driver is the charging authority by default
(`autonomous_mode:=false`). Do not enable robot-local autonomous charging at
the same time as the virtual driver.

The launch enables the front ToF sensor automatically for Daisy and leaves it
disabled for Lucas. Override that hardware profile when needed:

```bash
roslaunch vpa_robot_operation duckierace_start.launch use_tof:=true
```

The ToF launch must remain directly inside the robot namespace so its topics
resolve to `/<robot>/front_range` and `/<robot>/front_range_status`.

The main-PC virtual driver uses that range for anti-collision control in both
conservative and aggressive modes. Conservative mode slows below 0.55 m and
brakes at 0.30 m; aggressive mode slows below 0.45 m and brakes at 0.18 m,
with time-to-collision able to brake either profile earlier. These thresholds
are launch arguments of `vpa_duckierace/virtual_driver.launch`. A robot without
a live `front_range` topic can only use the supervisor when hardware is added.
The default virtual-driver launch therefore requires live data for Daisy and
fails safely if it becomes stale, while Lucas remains optional so its known
sensor-less hardware profile can run. Give Lucas ToF hardware, launch it with
`use_tof:=true`, and set `lucas_anticollision_sensor_required:=true` for the
same fail-safe protection.

Driver/charging and collision brakes are published as separate latched owner
states and ORed again at the wheel-driver boundary. This keeps an active safety
owner effective even if a diagnostic or keyboard tool writes to `local_brake`.

