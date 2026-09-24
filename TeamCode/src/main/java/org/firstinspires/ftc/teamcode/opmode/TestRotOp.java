package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.control.DriveProfile;
import org.firstinspires.ftc.teamcode.control.MoveToResult;
import org.firstinspires.ftc.teamcode.util.LoopTimer;

@Config
@TeleOp(name = "TestRot", group = "Debug")
public class TestRotOp extends OpMode {
    private static final double MAX_LOOP_DT_SECONDS = 0.1;

    private final LoopTimer loopTimer = new LoopTimer(MAX_LOOP_DT_SECONDS);
    private Robot robot;
    public static double kPX = Robot.DEFAULT_ROTATION_KP_POWER_PER_RADIAN;
    public static double kIX = Robot.DEFAULT_ROTATION_KI_POWER_PER_RADIAN_SECOND;
    public static double kDX = Robot.DEFAULT_ROTATION_KD_POWER_SECOND_PER_RADIAN;
    public static double rotationRateDegPerSecond = 90.0;
    public static double maxRotationPower = 0.60;
    public static double headingToleranceDeg = 2.0;
    public static double targetRot = 0;
    private boolean moving = false;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(
                telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot(hardwareMap);
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

        robot.setRotationPid(kPX, kIX, kDX);

        if (moving) {
            Pose2D targetPose = new Pose2D(
                    DistanceUnit.MM, 0, 0,
                    AngleUnit.DEGREES, targetRot);
            DriveProfile rotationProfile = DriveProfile.named("rotation-test")
                    .maxTranslationPower(0.0)
                    .maxRotationPower(safePower(maxRotationPower))
                    .positionToleranceMm(Double.MAX_VALUE)
                    .headingToleranceDeg(safeNonNegative(headingToleranceDeg))
                    .settleTimeMs(0.0)
                    .timeoutMs(0.0)
                    .build();
            MoveToResult result = robot.moveTo(targetPose, rotationProfile, dtSeconds);
            telemetry.addData("Heading error", "%.2f deg",
                    Math.toDegrees(result.getHeadingErrorRadians()));
            telemetry.addData("Rotation command", "%.3f",
                    result.getRequestedSignal().getCounterclockwise());
        } else {
            robot.stopDrive();
            robot.resetMoveToControllers();
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

        targetRot = AngleUnit.normalizeDegrees(
                targetRot
                        + applyDeadband(-gamepad1.left_stick_x)
                        * rotationRateDegPerSecond * dtSeconds);

        telemetry.addData("Rotation", robot.getHeadingDeg());
        telemetry.addData("KP", kPX);
        telemetry.addData("KI", kIX);
        telemetry.addData("KD", kDX);
        telemetry.addData("Target Rot", AngleUnit.normalizeDegrees(targetRot));
    }

    @Override
    public void stop() {
        robot.stop();
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
}
