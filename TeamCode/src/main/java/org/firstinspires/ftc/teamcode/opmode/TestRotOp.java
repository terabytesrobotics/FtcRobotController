package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "TestRot", group = "Debug")
public class TestRotOp extends OpMode {
    Robot robot;
    private double kPX, kIX, kDX;
    private double targetRot = 0;
    private boolean moving = false;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        kPX = robot.rDriveController.kP;
        kIX = robot.rDriveController.kI;
        kDX = robot.rDriveController.kD;
        
        robot.update();

        if (gamepad1.yWasPressed()) {
            moving = !moving;
        }

        if (moving) {
            robot.rotateTo(Math.toRadians(targetRot), true);
        }

        if (gamepad1.dpadUpWasPressed()) {
            if (gamepad1.a) {
                kPX *= 1.05;
            }
            if (gamepad1.b) {
                kIX *= 1.05;
            }

            if (gamepad1.x) {
                kDX *= 1.05;
            }
        }

        if (gamepad1.dpadDownWasPressed()) {
            if (gamepad1.a) {
                kPX /= 1.05;
            }

            if (gamepad1.b) {
                kIX /= 1.05;
            }

            if (gamepad1.x) {
                kDX /= 1.05;
            }
        }

        targetRot += applyDeadband(-gamepad1.left_stick_x);
//        targetY += applyDeadband(gamepad1.left_stick_y);

//        targetX = Math.min(-300.0, targetX);
//        targetX = Math.max(300.0, targetX);

//        targetY = Math.min(-300.0, targetY);
//        targetY = Math.max(300.0, targetY);

        robot.rDriveController.kP = kPX;
        robot.rDriveController.kI = kIX;
        robot.rDriveController.kD = kDX;

        telemetry.addData("Rotation", robot.getHeadingDeg());
        telemetry.addData("KP", kPX);
        telemetry.addData("KI", kIX);
        telemetry.addData("KD", kDX);
        telemetry.addData("Target Rot", AngleUnit.normalizeDegrees(targetRot));
    }

    private double applyDeadband(double value) {
        return Math.abs(value) < 0.05 ? 0.0 : Range.clip(value, -1.0, 1.0);
    }
}
