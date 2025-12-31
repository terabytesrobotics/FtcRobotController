# Project Overview
- FTC robot controller codebase focused on Road Runner–driven mecanum control with AprilTag localization and sample detection.
- Main control entry point is `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/DecodeRobotControl.java`, which wires hardware (pinpoint odometry, webcam, color sensor, shooter wheel, lift servo, mecanum drive) and coordinates teleop/autonomous behavior.
- `DecodeRobotControl` is constructed and driven only from `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/DecodeOpMode.java`, which initializes the robot, starts camera streaming to the dashboard, and repeatedly calls `evaluate()` while the op mode is active.
- Current loop handles manual driving, command-sequence execution, AprilTag-based pose updates, and dashboard telemetry logging/streaming; many TODOs remain and the system is a WIP baseline for future functionality.
- Robotics/game-specific configs live in `Constants` and related utilities (e.g., `AllianceColor`, `OnActivatedEvaluator`); extend these as new mechanisms and autonomous plans are added.
