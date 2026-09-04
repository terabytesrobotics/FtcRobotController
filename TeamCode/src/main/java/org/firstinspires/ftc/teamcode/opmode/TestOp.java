package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "PinpointTest", group = "Starter Bot")
public class TestOp extends OpMode {
    Robot robot;
    private double kPX, kIX, kDX;
    private double targetX = 0, targetY = 0;
    private boolean moving = false;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        kPX = robot.xDriveController.kP;
        kIX = robot.xDriveController.kI;
        kDX = robot.xDriveController.kD;
        
        robot.update();

        if (gamepad1.yWasPressed()) {
            moving = !moving;
        }

        if (moving) {
            robot.moveTo(targetX, targetY);
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

        targetX += applyDeadband(-gamepad1.left_stick_x) * 3;
//        targetY += applyDeadband(gamepad1.left_stick_y);

//        targetX = Math.min(-300.0, targetX);
//        targetX = Math.max(300.0, targetX);

//        targetY = Math.min(-300.0, targetY);
//        targetY = Math.max(300.0, targetY);

        robot.xDriveController.kP = kPX;
        robot.xDriveController.kI = kIX;
        robot.xDriveController.kD = kDX;

        telemetry.addData("X coordinate", robot.getX());
        telemetry.addData("Y coordinate", robot.getY());
        telemetry.addData("KP", kPX);
        telemetry.addData("KI", kIX);
        telemetry.addData("KD", kDX);
        telemetry.addData("Target X", targetX);
        telemetry.addData("Target Y", targetY);
//        telemetry.addData("Heading angle (deg)", robot.getHeading());
    }

    private double applyDeadband(double value) {
        return Math.abs(value) < 0.05 ? 0.0 : Range.clip(value, -1.0, 1.0);
    }
}
