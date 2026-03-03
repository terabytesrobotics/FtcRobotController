# Project Overview
- FTC robot controller codebase with Road Runner mecanum drive, shooter velocity control, toroid indexer control, and AprilTag localization in teleop.
- Main control entry point is `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/DecodeRobotControl.java`.
- Op mode entry point is `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/DecodeOpMode.java`, which constructs the controller and repeatedly calls `evaluate()`.
- Gamepad split: `gamepad1` drives (`translation/heading`), `gamepad2` runs mechanisms (`shooter`, `intake`, `lift`, `paddle/toroid`).

# Canonical Naming
- Use **paddle** for the mechanism and motor commonly called toroid/coreHex in code.
- `paddle` = `toroid` = hardware motor named `coreHex` in the RC config.
- Keep `coreHex` as the hardware-map string until the RC config is intentionally renamed.
- Older terms (`spindexer`, `kicker`, slot-servo language) are obsolete for the current op mode and should not be used in new notes.

# Canonical Hardware Map (Current)
- `pinpoint`: GoBilda pinpoint odometry driver.
- `Webcam 1`: camera used for AprilTags + sample detection pipeline.
- `wheel`: shooter wheel motor (`DcMotorEx`, velocity-controlled, float at zero power).
- `intake`: intake motor (`DcMotorEx`).
- `coreHex`: paddle/toroid motor (`DcMotorEx`).
- `color1`: paddle-facing `RevColorSensorV3` used for paddle/ball observation and zero confidence updates.

# Paddle/CoreHex Notes
- Current code symbol is `TOROID_MOTOR_NAME = "coreHex"` in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/DecodeRobotControl.java`.
- Paddle manual command is on `gamepad2.right_stick_y` with deadband and RPM caps.
- Autonomous paddle shooting uses step-based motion (`TOROID_SHOOT_STEPS`) and jam handling.
- Persistent paddle zero estimate is stored in `toroid_zero.txt`.

# Magnetic Limit Switch
- A magnetic limit switch is now part of the physical robot for the paddle mechanism.
- Software integration should use the name **paddle limit switch** in docs/comments to match pit-language.
- Hardware-map name is `mag`.
- Control intent: when `gamepad2.right_stick_y` is in deadzone, paddle auto-seeks home until `mag` is seen, then holds home; stick input still allows normal bidirectional manual motion for transit/shoot.
- Homing records coreHex encoder ticks at switch press with debounce, using a tunable home offset constant in degrees (`TOROID_HOME_OFFSET_DEGREES`).

# Other Behavior (Current)
- Teleop localization uses AprilTag detections with queue/variance gating before applying pose updates.
- Autonomous currently does not fuse AprilTag updates.
- Intake control is manual-first in teleop: hold `gamepad2.x` to stop, `gamepad2.dpad_down` for reverse, otherwise forward when enabled.
