package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "PinpointTest", group = "Starter Bot")
public class TestOp extends OpMode {
    Robot robot;
    private double kP = 0, kI = 0, kD = 0;
    private double kPDelta, kIDelta, kDDelta;
    private double targetX = 0, targetY = 0;
    private boolean moving = false;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        robot.update();

        if (gamepad1.yWasPressed()) {
            moving = !moving;
        }

        if (moving) {
            robot.moveTo(targetX, targetY);
        }

        if (gamepad1.dpadUpWasPressed()) {
            if (gamepad1.a) {
                kP += kPDelta;
            }
            if (gamepad1.b) {
                kI += kIDelta;
            }

            if (gamepad1.x) {
                kD += kDDelta;
            }
        }

        if (gamepad1.dpadDownWasPressed()) {
            if (gamepad1.a) {
                kP -= kPDelta;
            }
            if (gamepad1.b) {
                kI -= kIDelta;
            }

            if (gamepad1.x) {
                kD -= kDDelta;
            }
        }

        targetX += applyDeadband(-gamepad1.left_stick_y);
        targetY += applyDeadband(gamepad1.left_stick_x);

        targetX = Math.min(-300.0, targetX);
        targetX = Math.max(300.0, targetX);

        targetY = Math.min(-300.0, targetY);
        targetY = Math.max(300.0, targetY);

        robot.driveTrainController.kP = kP;
        robot.driveTrainController.kI = kI;
        robot.driveTrainController.kD = kD;

        telemetry.addData("X coordinate", robot.getX());
        telemetry.addData("Y coordinate", robot.getY());
        telemetry.addData("KP", kP);
        telemetry.addData("KI", kI);
        telemetry.addData("KD", kD);
        telemetry.addData("Target X", targetX);
        telemetry.addData("Target Y", targetY);
//        telemetry.addData("Heading angle (deg)", robot.getHeading());
    }

    private double applyDeadband(double value) {
        return Math.abs(value) < 0.05 ? 0.0 : Range.clip(value, -1.0, 1.0);
    }
}
