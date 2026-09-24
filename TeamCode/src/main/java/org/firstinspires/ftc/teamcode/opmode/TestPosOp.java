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
import org.firstinspires.ftc.teamcode.control.DriveProfile;
import org.firstinspires.ftc.teamcode.control.DriveSignal;
import org.firstinspires.ftc.teamcode.control.MoveToResult;
import org.firstinspires.ftc.teamcode.control.WheelPowers;
import org.firstinspires.ftc.teamcode.util.LoopTimer;

@Config
@TeleOp(name = "TestPose (Carrot)", group = "Debug")
public class TestPosOp extends OpMode {
    private static final double MAX_LOOP_DT_SECONDS = 0.1;
    private static final double MM_PER_INCH = 25.4;
    private static final double ROBOT_RADIUS_INCHES = 9.0;
    private static final double TARGET_RADIUS_INCHES = 3.0;

    private final LoopTimer loopTimer = new LoopTimer(MAX_LOOP_DT_SECONDS);
    private Robot robot;
    private FtcDashboard dashboard;
    private MoveToResult moveToResult;

    public static double kPX = Robot.DEFAULT_TRANSLATION_KP_POWER_PER_MM;
    public static double kIX = Robot.DEFAULT_TRANSLATION_KI_POWER_PER_MM_SECOND;
    public static double kDX = Robot.DEFAULT_TRANSLATION_KD_POWER_SECOND_PER_MM;
    public static double maxTranslationPower = 0.75;
    public static double maxRotationPower = 0.60;
    public static double positionToleranceMm = 8.0;
    public static double headingToleranceDeg = 2.0;
    public static double translationRateMmPerSecond = 300;
    public static double rotationRateDegPerSecond = 90;
    public static double targetX = 0;
    public static double targetY = 0;
    public static double targetHeadingDeg = 0;
    private boolean moving = true;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        robot = new Robot(hardwareMap);

        telemetry.addLine("Left stick: move target on field");
        telemetry.addLine("Right stick X: rotate target");
        telemetry.addLine("Y: follow/pause | A: target current pose");
    }

    @Override
    public void start() {
        loopTimer.reset();
        robot.resetMoveToControllers();
    }

    @Override
    public void loop() {
        double dtSeconds = loopTimer.nextSeconds();
        robot.updateSensors();

        if (gamepad1.yWasPressed()) {
            moving = !moving;
        }

        if (gamepad1.aWasPressed()) {
            targetX = robot.getX();
            targetY = robot.getY();
            targetHeadingDeg = robot.getHeadingDeg();
            robot.resetMoveToControllers();
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

        robot.setTranslationPid(kPX, kIX, kDX);

        if (moving) {
            Pose2D targetPose = new Pose2D(
                    DistanceUnit.MM, targetX, targetY,
                    AngleUnit.DEGREES, targetHeadingDeg);
            DriveProfile carrotProfile = DriveProfile.named("carrot")
                    .maxTranslationPower(safePower(maxTranslationPower))
                    .maxRotationPower(safePower(maxRotationPower))
                    .positionToleranceMm(safeNonNegative(positionToleranceMm))
                    .headingToleranceDeg(safeNonNegative(headingToleranceDeg))
                    .settleTimeMs(0.0)
                    .timeoutMs(0.0)
                    .build();
            moveToResult = robot.moveTo(targetPose, carrotProfile, dtSeconds);
        } else {
            moveToResult = null;
            robot.stopDrive();
            robot.resetMoveToControllers();
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
        if (moveToResult != null) {
            DriveSignal signal = moveToResult.getRequestedSignal();
            WheelPowers wheels = moveToResult.getWheelPowers();
            telemetry.addData("Field error X/Y", "%.1f / %.1f mm",
                    moveToResult.getFieldXErrorMm(), moveToResult.getFieldYErrorMm());
            telemetry.addData("Robot error forward/left", "%.1f / %.1f mm",
                    moveToResult.getRobotForwardErrorMm(), moveToResult.getRobotLeftErrorMm());
            telemetry.addData("Heading error", "%.2f deg",
                    Math.toDegrees(moveToResult.getHeadingErrorRadians()));
            telemetry.addData("Drive forward/left/CCW", "%.3f / %.3f / %.3f",
                    signal.getForward(), signal.getLeft(), signal.getCounterclockwise());
            telemetry.addData("Wheels FL/FR/BL/BR", "%.3f / %.3f / %.3f / %.3f",
                    wheels.getFrontLeft(), wheels.getFrontRight(),
                    wheels.getBackLeft(), wheels.getBackRight());
        }

        sendDashboardFieldOverlay();
    }

    private double applyDeadband(double value) {
        return Math.abs(value) < 0.05 ? 0.0 : Range.clip(value, -1.0, 1.0);
    }

    private double safePower(double value) {
        return Double.isFinite(value) ? Range.clip(value, 0.0, 1.0) : 0.0;
    }

    private double safeNonNegative(double value) {
        return Double.isFinite(value) ? Math.max(0.0, value) : 0.0;
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

    @Override
    public void stop() {
        robot.stop();
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
