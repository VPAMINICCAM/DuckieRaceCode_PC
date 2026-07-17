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

