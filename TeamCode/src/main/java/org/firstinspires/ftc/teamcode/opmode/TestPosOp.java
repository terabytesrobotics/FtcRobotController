package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot;

@Config
@TeleOp(name = "TestPose (Carrot)", group = "Debug")
public class TestPosOp extends OpMode {
    private static final double MAX_LOOP_DT_SECONDS = 0.1;
    private static final double MM_PER_INCH = 25.4;
    private static final double ROBOT_RADIUS_INCHES = 9.0;
    private static final double TARGET_RADIUS_INCHES = 3.0;

    private Robot robot;
    private FtcDashboard dashboard;
    private long lastLoopTimeNanos;

    public static double kPX = 0.006;
    public static double kIX = 0.0006;
    public static double kDX = 0.02;
    public static double translationRateMmPerSecond = 300;
    public static double rotationRateDegPerSecond = 90;
    public static double targetX = 0;
    public static double targetY = 0;
    public static double targetHeadingDeg = 0;
    private boolean moving = true;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        robot = new Robot(hardwareMap, telemetry);

        telemetry.addLine("Left stick: move target on field");
        telemetry.addLine("Right stick X: rotate target");
        telemetry.addLine("Y: follow/pause | A: target current pose");
    }

    @Override
    public void start() {
        lastLoopTimeNanos = System.nanoTime();
    }

    @Override
    public void loop() {
        long nowNanos = System.nanoTime();
        double dtSeconds = Range.clip(
                (nowNanos - lastLoopTimeNanos) / 1_000_000_000.0,
                0.0,
                MAX_LOOP_DT_SECONDS);
        lastLoopTimeNanos = nowNanos;

        robot.update();

        if (gamepad1.yWasPressed()) {
            moving = !moving;
        }

        if (gamepad1.aWasPressed()) {
            targetX = robot.getX();
            targetY = robot.getY();
            targetHeadingDeg = robot.getHeadingDeg();
        }

        // Move the carrot in field coordinates at rates that do not depend on loop speed.
        targetX += applyDeadband(-gamepad1.left_stick_y)
                * translationRateMmPerSecond * dtSeconds;
        targetY += applyDeadband(-gamepad1.left_stick_x)
                * translationRateMmPerSecond * dtSeconds;
        targetHeadingDeg = AngleUnit.normalizeDegrees(
                targetHeadingDeg
                        - applyDeadband(gamepad1.right_stick_x)
                        * rotationRateDegPerSecond * dtSeconds);

        robot.xDriveController.kP = kPX;
        robot.xDriveController.kI = kIX;
        robot.xDriveController.kD = kDX;
        robot.yDriveController.kP = kPX;
        robot.yDriveController.kI = kIX;
        robot.yDriveController.kD = kDX;

        if (moving) {
            Pose2D targetPose = new Pose2D(
                    DistanceUnit.MM, targetX, targetY,
                    AngleUnit.DEGREES, targetHeadingDeg);
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

        telemetry.addData("X coordinate", robot.getX());
        telemetry.addData("Y coordinate", robot.getY());
        telemetry.addData("KP", kPX);
        telemetry.addData("KI", kIX);
        telemetry.addData("KD", kDX);
        telemetry.addData("Following carrot", moving);
        telemetry.addData("Loop dt", "%.3f s", dtSeconds);
        telemetry.addData("Target X/Y", "%.1f / %.1f mm", targetX, targetY);
        telemetry.addData("Target heading", "%.1f deg", targetHeadingDeg);

        sendDashboardFieldOverlay();
    }

    private double applyDeadband(double value) {
        return Math.abs(value) < 0.05 ? 0.0 : Range.clip(value, -1.0, 1.0);
    }

    private void sendDashboardFieldOverlay() {
        double currentXInches = robot.getX() / MM_PER_INCH;
        double currentYInches = robot.getY() / MM_PER_INCH;
        double currentHeadingRadians = robot.getHeadingRad();
        double targetXInches = targetX / MM_PER_INCH;
        double targetYInches = targetY / MM_PER_INCH;
        double targetHeadingRadians = Math.toRadians(targetHeadingDeg);

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("followingCarrot", moving);
        packet.put("currentXmm", robot.getX());
        packet.put("currentYmm", robot.getY());
        packet.put("currentHeadingDeg", robot.getHeadingDeg());
        packet.put("targetXmm", targetX);
        packet.put("targetYmm", targetY);
        packet.put("targetHeadingDeg", targetHeadingDeg);

        Canvas field = packet.fieldOverlay();
        field.setStroke("#4CAF50");
        field.strokeLine(currentXInches, currentYInches, targetXInches, targetYInches);
        drawPose(field, currentXInches, currentYInches, currentHeadingRadians,
                ROBOT_RADIUS_INCHES, "#2196F3");
        drawPose(field, targetXInches, targetYInches, targetHeadingRadians,
                TARGET_RADIUS_INCHES, "#FF9800");

        dashboard.sendTelemetryPacket(packet);
    }

    private void drawPose(Canvas field, double x, double y, double heading,
                          double radius, String color) {
        field.setStroke(color);
        field.strokeCircle(x, y, radius);
        field.strokeLine(
                x,
                y,
                x + Math.cos(heading) * radius,
                y + Math.sin(heading) * radius);
    }
}
