# Starter Mecanum Drive

This project starts from the official FIRST Tech Challenge Robot Controller SDK. The first
team OpMode is deliberately limited to the four-wheel mecanum drivetrain so mechanical,
wiring, configuration, and control issues can be isolated before game mechanisms are added.

## Robot Configuration

Create or edit the active configuration on the Driver Station so these four names match
exactly:

| Robot position | Configuration name |
| --- | --- |
| Front left | `front_left_drive` |
| Front right | `front_right_drive` |
| Back left | `back_left_drive` |
| Back right | `back_right_drive` |
| Center collector roller motor | `center_collector` |
| Left side intake CR servo | `left_intake` |
| Right side intake CR servo | `right_intake` |

Use the actual Control Hub/Expansion Hub ports chosen during wiring. The Java names identify
the motors; they do not require a particular port order.

## Driver Controls

| Input | Action |
| --- | --- |
| Left stick up/down | Drive forward/backward |
| Left stick left/right | Strafe left/right |
| Right stick left/right | Rotate counterclockwise/clockwise |
| Hold left bumper | Precision mode (30%) |
| No bumper | Normal mode (60%) |
| Hold right bumper | Full-speed mode (100%) |
| Right trigger | Run all collector wheels inward |
| Left trigger | Reverse all collector wheels to eject |

If both bumpers are held, precision mode wins.
If both triggers are pressed, their commands subtract from one another.

## First Test

1. Install the four mecanum wheels so their rollers form an `X` when viewed from above.
2. Put the robot on a sturdy stand with all wheels clear of the table and floor.
3. Confirm the active Robot Configuration contains all four names above.
4. Select `Starter: Mecanum Drive`, initialize it, and keep hands clear of the drivetrain.
5. Press START and use small stick movements in normal or precision mode.
6. Check forward, backward, rotation, and strafing one at a time before combining them.
7. Test the collector slowly with the right trigger. The center roller and both side wheels
   should all move game pieces inward without fighting each other.
8. Test ejection slowly with the left trigger.
9. Put the robot on the floor only after every isolated movement is correct.

For a forward command, all four wheels must push the robot forward. If every wheel moves the
robot backward, swap `FORWARD` and `REVERSE` for all four motors in
`StarterMecanumTeleOp.java`. If only one wheel is wrong, first verify that the Robot
Configuration name refers to the motor in that physical corner, then correct only that motor's
direction if necessary.

The two CR servos are intentionally configured with opposite directions because their side
wheels are mirrored. If the entire collector runs backward, reverse the center motor and swap
both servo directions. If only one element is wrong, change only that device's direction.

## Intended Next Iterations

- Tune `NORMAL_SPEED`, `PRECISION_SPEED`, and driver stick preferences.
- Adjust `STRAFE_MULTIPLIER` only after testing on regulation foam tiles.
- Add field-relative driving after the Control Hub mounting orientation is final.
- Add mechanisms in separate hardware/subsystem classes rather than expanding this file into
  a season-long monolith.
