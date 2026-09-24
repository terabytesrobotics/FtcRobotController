# TeamCode control architecture

The FTC `OpMode` classes are adapters, not the robot implementation.

```text
FTC OpMode
  -> TeleopController or SequentialCommandRunner
    -> RobotActions / Robot
      -> Drive, Collector, Pinpoint (and later optional Vision)
```

## Responsibilities

- `Robot` owns configured hardware, sensor refresh, and shared behaviors. It must not read gamepads,
  select autonomous plans, or publish telemetry.
- `TeleopController` translates gamepad input into `DriveSignal` and collector requests.
- Autonomous plans live under `autonomous/` and compose typed commands from `command/`.
- Op modes own lifecycle, loop timing, Driver Station telemetry, and Dashboard presentation.
- Limelight remains a standalone diagnostic until it is added as an optional subsystem. A missing
  camera must not prevent drivetrain and collector testing.

## Drive conventions

- `DriveSignal`: positive forward, positive left, positive counterclockwise.
- `Pose2D`: millimeters and radians internally unless the constructor explicitly says otherwise.
- `DriveProfile`: powers, position tolerance in millimeters, heading tolerance in degrees at the
  builder boundary, and settle/timeout values in milliseconds at the builder boundary.
- `Robot.moveTo(...)` is nonblocking and returns `MoveToResult`; callers decide how to display it or
  when a sequence has settled long enough to advance.

## Autonomous expression

Critical motion behavior belongs in a named `DriveProfile`, not a positional command constructor:

```java
DriveProfile travel = DriveProfile.named("travel")
        .maxTranslationPower(0.45)
        .maxRotationPower(0.35)
        .positionToleranceMm(20.0)
        .headingToleranceDeg(4.0)
        .settleTimeMs(200.0)
        .timeoutMs(5000.0)
        .build();

return SequentialCommandRunner.sequence(
        DriveToCommand.to(destination).using(travel),
        SetCollectorCommand.to(CollectorMode.INTAKE));
```

Every long-running command must return a terminal status and include a timeout where physical
completion is not guaranteed.
