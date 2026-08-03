/*
 * Starter drivetrain TeleOp for the Terabytes 2026-2027 robot.
 *
 * This is intentionally robot-centric and drivetrain-only. It gives the team a small,
 * dependable starting point for checking wheel placement, motor direction, wiring, and
 * driver preference before adding mechanisms or field-relative control.
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Starter: Mecanum Drive", group = "Starter Bot")
public class StarterMecanumTeleOp extends OpMode {
    // These names must exactly match the active Robot Configuration on the Driver Station.
    private static final String FRONT_LEFT_NAME = "front_left_drive";
    private static final String FRONT_RIGHT_NAME = "front_right_drive";
    private static final String BACK_LEFT_NAME = "back_left_drive";
    private static final String BACK_RIGHT_NAME = "back_right_drive";

    // Start conservatively. Hold a bumper to temporarily select another speed.
    private static final double NORMAL_SPEED = 0.60;
    private static final double PRECISION_SPEED = 0.30;
    private static final double FULL_SPEED = 1.00;
    private static final double STICK_DEADBAND = 0.05;

    // Leave at 1.0 for initial testing. This can later compensate for imperfect strafing.
    private static final double STRAFE_MULTIPLIER = 1.00;

    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotorEx.class, FRONT_LEFT_NAME);
        frontRight = hardwareMap.get(DcMotorEx.class, FRONT_RIGHT_NAME);
        backLeft = hardwareMap.get(DcMotorEx.class, BACK_LEFT_NAME);
        backRight = hardwareMap.get(DcMotorEx.class, BACK_RIGHT_NAME);

        // This matches the current official FTC mecanum samples. Verify it with the wheels
        // raised before driving; gearing or motor placement may require flipping all four.
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        configureMotor(frontLeft);
        configureMotor(frontRight);
        configureMotor(backLeft);
        configureMotor(backRight);

        telemetry.addLine("Starter mecanum drive initialized");
        telemetry.addLine("Left stick: drive/strafe | Right stick X: rotate");
        telemetry.addLine("Left bumper: precision | Right bumper: full speed");
        telemetry.addLine("Before START: raise the wheels and verify roller/motor directions");
    }

    @Override
    public void loop() {
        double forward = applyDeadband(-gamepad1.left_stick_y);
        double strafe = applyDeadband(gamepad1.left_stick_x) * STRAFE_MULTIPLIER;
        double rotate = applyDeadband(gamepad1.right_stick_x);

        // A mecanum drivetrain combines forward, strafe, and rotation at every wheel.
        double frontLeftPower = forward + strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backLeftPower = forward - strafe + rotate;
        double backRightPower = forward + strafe - rotate;

        // Normalize as a group so the requested movement direction is preserved.
        double denominator = Math.max(
                Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1.0);
        double speed = selectedSpeed();

        frontLeftPower = speed * frontLeftPower / denominator;
        frontRightPower = speed * frontRightPower / denominator;
        backLeftPower = speed * backLeftPower / denominator;
        backRightPower = speed * backRightPower / denominator;

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        telemetry.addData("Speed limit", "%.0f%%", speed * 100.0);
        telemetry.addData("Command F/S/R", "%+.2f / %+.2f / %+.2f", forward, strafe, rotate);
        telemetry.addData("Front L/R", "%+.2f / %+.2f", frontLeftPower, frontRightPower);
        telemetry.addData("Back L/R", "%+.2f / %+.2f", backLeftPower, backRightPower);
    }

    private void configureMotor(DcMotorEx motor) {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(0.0);
    }

    private double applyDeadband(double value) {
        return Math.abs(value) < STICK_DEADBAND ? 0.0 : Range.clip(value, -1.0, 1.0);
    }

    private double selectedSpeed() {
        if (gamepad1.left_bumper) {
            return PRECISION_SPEED;
        }
        if (gamepad1.right_bumper) {
            return FULL_SPEED;
        }
        return NORMAL_SPEED;
    }
}
