package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "TestPos", group = "Debug")
public class TestPosOp extends OpMode {
    Robot robot;
    private double kPX, kIX, kDX;
    private double targetX = 0, targetY = 0, targetRot = 0;
    private boolean moving = false;
    private final ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);

        robot.setPos(new Pose2D(DistanceUnit.INCH, -76 + 9, 76 - 7, AngleUnit.DEGREES, -90));
        robot.update();
        targetX = robot.getX();
        targetY = robot.getY();
        targetRot = robot.getHeadingRad();
    }

    @Override
    public void loop() {
        double dtSeconds = loopTimer.seconds();
        loopTimer.reset();

//        kPX = robot.xDriveController.kP;
//        kIX = robot.xDriveController.kI / 1E-5;
//        kDX = robot.xDriveController.kD;
        kPX = robot.rDriveController.kP;
        kIX = robot.rDriveController.kI / 1E-5;
        kDX = robot.rDriveController.kD;
        
        robot.update();

        if (gamepad1.yWasPressed()) {
            moving = !moving;
        }

//        targetX += applyDeadband(-gamepad1.left_stick_x) * 5;
//        targetY += applyDeadband(gamepad1.left_stick_y) * 5;
        targetX += applyDeadband(-gamepad1.left_stick_y) * 8;
        targetY += applyDeadband(-gamepad1.left_stick_x) * 8;
        targetRot += applyDeadband(gamepad1.right_stick_x) / 15;

        if (moving) {
//            robot.moveTo(targetX, targetY, true);
            robot.moveToRot(targetX, targetY, targetRot, dtSeconds, true);
//            robot.rotateTo(Math.PI, true);
        } else {
            robot.drive.stop();
        }

        if (gamepad1.dpadUpWasPressed()) {
            if (gamepad1.a) {
                kPX *= 1.1;

                if (kPX == 0) {
                    kPX = 0.001;
                }
            }
            if (gamepad1.b) {
                kIX *= 1.1;

                if (kIX == 0) {
                    kIX = .5;
                }
            }

            if (gamepad1.x) {
                kDX *= 1.1;

                if (kDX == 0) {
                    kDX = 0.0001;
                }
            }
        }

        if (gamepad1.dpadDownWasPressed()) {
            if (gamepad1.a) {
                kPX /= 1.1;

                if (kPX < 0.001) {
                    kPX = 0;
                }
            }

            if (gamepad1.b) {
                kIX /= 1.1;

                if (kIX < .5) {
                    kIX = 0;
                }
            }

            if (gamepad1.x) {
                kDX /= 1.1;

                if (kDX < 0.0001) {
                    kDX = 0;
                }
            }
        }

//        targetX = Math.min(-300.0, targetX);
//        targetX = Math.max(300.0, targetX);

//        targetY = Math.min(-300.0, targetY);
//        targetY = Math.max(300.0, targetY);

//        robot.xDriveController.kP = kPX;
//        robot.xDriveController.kI = kIX * 1E-5;
//        robot.xDriveController.kD = kDX;
//        robot.yDriveController.kP = kPX;
//        robot.yDriveController.kI = kIX * 1E-5;
//        robot.yDriveController.kD = kDX;
        robot.rDriveController.kP = kPX;
        robot.rDriveController.kI = kIX * 1E-5;
        robot.rDriveController.kD = kDX;

        robot.telemetry.addData("X coordinate", robot.getX());
        robot.telemetry.addData("Y coordinate", robot.getY());
        robot.telemetry.addData("Rot", robot.getHeadingRad());
        robot.telemetry.addData("KP", kPX);
        robot.telemetry.addData("KI", kIX);
        robot.telemetry.addData("KD", kDX);
        robot.telemetry.addData("Target X", targetX);
        robot.telemetry.addData("Target Y", targetY);
        robot.telemetry.addData("Target Rot", targetRot);
//        telemetry.addData("Heading angle (deg)", robot.getHeading());
    }

    private double applyDeadband(double value) {
        return Math.abs(value) < 0.05 ? 0.0 : Range.clip(value, -1.0, 1.0);
    }
}
