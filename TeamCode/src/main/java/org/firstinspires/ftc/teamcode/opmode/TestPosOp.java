package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robot;

@Config("Pose Test")
@TeleOp(name = "TestPos", group = "Debug")
public class TestPosOp extends OpMode {
    private static final double MM_PER_INCH = 25.4;
    private static final double ROBOT_RADIUS_INCHES = 9.0;

    Robot robot;
    private FtcDashboard dashboard;
    public static volatile double kPX = 5.4;
    public static volatile double kIX = 0.0;
    public static volatile double kDX = 0.21;
    private double targetX = 0, targetY = 0, targetRot = 0;
    private boolean moving = false;
    private final ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        robot = new Robot(hardwareMap);
    }

    @Override
    public void loop() {
        double dtSeconds = loopTimer.seconds();
        loopTimer.reset();

        robot.update();

        if (gamepad1.yWasPressed()) {
            moving = !moving;
        }

//        targetX += applyDeadband(-gamepad1.left_stick_x) * 5;
//        targetY += applyDeadband(gamepad1.left_stick_y) * 5;
        targetX += applyDeadband(-gamepad1.left_stick_y) * 8;
        targetY += applyDeadband(-gamepad1.left_stick_x) * 8;
        targetRot += applyDeadband(gamepad1.right_stick_x) / 15;

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
                    kIX = 5E-6;
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

                if (kIX < 5E-6) {
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
        robot.rDriveController.kI = kIX;
        robot.rDriveController.kD = kDX;

        if (moving) {
            robot.moveToRot(targetX, targetY, targetRot, dtSeconds, true);
        } else {
            robot.drive.stop();
        }

        double currentX = robot.getX();
        double currentY = robot.getY();
        double currentHeading = robot.getHeadingRad();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("loop.dtMs", dtSeconds * 1000.0);
        packet.put("pose.xMm", currentX);
        packet.put("pose.yMm", currentY);
        packet.put("pose.headingDeg", Math.toDegrees(currentHeading));
        packet.put("target.xMm", targetX);
        packet.put("target.yMm", targetY);
        packet.put("target.headingDeg", Math.toDegrees(targetRot));
        packet.put("error.distanceMm", Math.hypot(targetX - currentX, targetY - currentY));
        packet.put("error.headingDeg", Math.toDegrees(
                org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians(
                        targetRot - currentHeading)));
        packet.put("following", moving);
        packet.put("pid.heading.kP", kPX);
        packet.put("pid.heading.kI", kIX);
        packet.put("pid.heading.kD", kDX);

        Canvas field = packet.fieldOverlay();
        drawPose(field, currentX / MM_PER_INCH, currentY / MM_PER_INCH,
                currentHeading, ROBOT_RADIUS_INCHES, "#2196F3");
        drawPose(field, targetX / MM_PER_INCH, targetY / MM_PER_INCH,
                targetRot, 3.0, "#FF9800");
        dashboard.sendTelemetryPacket(packet);

        telemetry.addData("Position (mm)", "%.0f, %.0f", currentX, currentY);
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(currentHeading));
        telemetry.addData("Target (mm)", "%.0f, %.0f", targetX, targetY);
        telemetry.addData("Following", moving);
        telemetry.update();
    }

    private double applyDeadband(double value) {
        return Math.abs(value) < 0.05 ? 0.0 : Range.clip(value, -1.0, 1.0);
    }

    private void drawPose(
            Canvas field,
            double x,
            double y,
            double heading,
            double radius,
            String color
    ) {
        field.setStroke(color);
        field.strokeCircle(x, y, radius);
        field.strokeLine(
                x,
                y,
                x + radius * Math.cos(heading),
                y + radius * Math.sin(heading));
    }
}
