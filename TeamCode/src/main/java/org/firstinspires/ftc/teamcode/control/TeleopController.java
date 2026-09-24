package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robot;

/** Maps driver intent to the same mode-independent Robot behaviors used by autonomous. */
public final class TeleopController {
    private static final double NORMAL_SPEED = 0.60;
    private static final double PRECISION_SPEED = 0.30;
    private static final double FULL_SPEED = 1.00;

    private final Gamepad gamepad;
    private final double deadband;
    private DriveSignal lastDriveSignal = DriveSignal.ZERO;
    private WheelPowers lastWheelPowers = WheelPowers.ZERO;
    private double lastCollectorPower;
    private double lastSpeed = NORMAL_SPEED;

    public TeleopController(Gamepad gamepad, double deadband) {
        this.gamepad = gamepad;
        this.deadband = deadband;
    }

    public void start(Robot robot) {
        robot.resetMoveToControllers();
    }

    public void update(Robot robot, double dtSeconds) {
        double forward = applyDeadband(-gamepad.left_stick_y);
        double left = applyDeadband(-gamepad.left_stick_x);
        double counterclockwise = applyDeadband(-gamepad.right_stick_x);

        lastSpeed = selectedSpeed();
        lastDriveSignal = new DriveSignal(
                forward * lastSpeed,
                left * lastSpeed,
                counterclockwise * lastSpeed);
        lastWheelPowers = robot.driveRobotRelative(lastDriveSignal);

        lastCollectorPower = applyDeadband(gamepad.right_trigger - gamepad.left_trigger);
        robot.setCollectorPower(lastCollectorPower);
    }

    public DriveSignal getLastDriveSignal() {
        return lastDriveSignal;
    }

    public WheelPowers getLastWheelPowers() {
        return lastWheelPowers;
    }

    public double getLastCollectorPower() {
        return lastCollectorPower;
    }

    public double getLastSpeed() {
        return lastSpeed;
    }

    private double applyDeadband(double value) {
        return Math.abs(value) < deadband ? 0.0 : Range.clip(value, -1.0, 1.0);
    }

    private double selectedSpeed() {
        if (gamepad.left_bumper) {
            return PRECISION_SPEED;
        }
        if (gamepad.right_bumper) {
            return FULL_SPEED;
        }
        return NORMAL_SPEED;
    }
}
