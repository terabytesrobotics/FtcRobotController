package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "TestPos", group = "Debug")
public class TestPosOp extends OpMode {
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

        targetX += applyDeadband(-gamepad1.left_stick_x) * 5;
        targetY += applyDeadband(gamepad1.left_stick_y) * 5;

        if (moving) {
            Pose2D targetPose = new Pose2D(
                    DistanceUnit.MM, targetX, targetY,
                    AngleUnit.RADIANS, 0);
            robot.moveTo(targetPose, true);
        }

        if (gamepad1.dpadUpWasPressed()) {
            if (gamepad1.a) {
                kPX *= 1.1;
            }
            if (gamepad1.b) {
                kIX *= 1.1;
            }

            if (gamepad1.x) {
                kDX *= 1.1;
            }
        }

        if (gamepad1.dpadDownWasPressed()) {
            if (gamepad1.a) {
                kPX /= 1.1;
            }

            if (gamepad1.b) {
                kIX /= 1.1;
            }

            if (gamepad1.x) {
                kDX /= 1.1;
            }
        }

//        targetX = Math.min(-300.0, targetX);
//        targetX = Math.max(300.0, targetX);

//        targetY = Math.min(-300.0, targetY);
//        targetY = Math.max(300.0, targetY);

        robot.xDriveController.kP = kPX;
        robot.xDriveController.kI = kIX;
        robot.xDriveController.kD = kDX;
        robot.yDriveController.kP = kPX;
        robot.yDriveController.kI = kIX;
        robot.yDriveController.kD = kDX;

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
