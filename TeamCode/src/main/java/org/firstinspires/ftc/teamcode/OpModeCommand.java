package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class OpModeCommand {

    private static final double MIN_TIME_STANDARD = 250;
    private static final double MIN_TIME_NONE = 0;
    private static final double SETTLE_TIME_STANDARD = 300;
    private static final double SETTLE_TIME_NONE = 0;
    private static final double PRECISE_SETTLE_RATIO = 0.68d;
    private static final double SETTLE_RATIO_STANDARD = 1d;
    private static final double SETTLE_RATIO_COARSE = 3d;
    private static final double SHOT_QUALITY_SETTLE_TIME = 100d;

    public final Integer WaitUntilElapsedMillis;
    public final Pose2d DriveToPose;
    public final Double MinTimeMillis;
    public final Double SettleTimeMillis;
    public final Double DriveSettleThresholdRatio;
    public final Double DrivePowerScale;
    public final Double IntakePower;
    public final Double IntakeHoldMillis;
    public final Boolean ShooterEnabled;
    public final Double ToroidShootRevolutions;
    public final Double ToroidTimeoutMillis;
    public final boolean RequireActionStarted;
    public final boolean RequireShooterEnabled;
    public final boolean RequireShotQuality;

    private OpModeCommand(
            @Nullable Integer waitUntilElapsedMillis,
            @Nullable Pose2d driveDirectToPose,
            @NonNull Double minTimeMillis,
            @NonNull Double settleTimeMillis,
            @NonNull Double settleThresholdRatio,
            @Nullable Double drivePowerScale,
            @Nullable Double intakePower,
            @Nullable Double intakeHoldMillis,
            @Nullable Boolean shooterEnabled,
            @Nullable Double toroidShootRevolutions,
            @Nullable Double toroidTimeoutMillis,
            boolean requireActionStarted,
            boolean requireShooterEnabled,
            boolean requireShotQuality) {
        WaitUntilElapsedMillis = waitUntilElapsedMillis;
        DriveToPose = driveDirectToPose;
        MinTimeMillis = minTimeMillis;
        SettleTimeMillis = settleTimeMillis;
        DriveSettleThresholdRatio = settleThresholdRatio;
        DrivePowerScale = drivePowerScale;
        IntakePower = intakePower;
        IntakeHoldMillis = intakeHoldMillis;
        ShooterEnabled = shooterEnabled;
        ToroidShootRevolutions = toroidShootRevolutions;
        ToroidTimeoutMillis = toroidTimeoutMillis;
        RequireActionStarted = requireActionStarted;
        RequireShooterEnabled = requireShooterEnabled;
        RequireShotQuality = requireShotQuality;
    }

    public OpModeCommand(
            @NonNull Integer waitUntilElapsedMillis,
            @Nullable Pose2d driveDirectToPose,
            @NonNull Double minTimeMillis,
            @NonNull Double settleTimeMillis,
            @NonNull Double settleThresholdRatio) {
        this(waitUntilElapsedMillis, driveDirectToPose, minTimeMillis, settleTimeMillis, settleThresholdRatio,
                null,
                null, null, null, null, null, false, false, false);
    }

    public OpModeCommand(
            @Nullable Pose2d driveDirectToPose,
            @NonNull Double minTimeMillis,
            @NonNull Double settleTimeMillis,
            @NonNull Double settleThresholdRatio) {
        this(0, driveDirectToPose, minTimeMillis, settleTimeMillis, settleThresholdRatio,
                null,
                null, null, null, null, null, false, false, false);
    }

    public OpModeCommand withWaitUntil(int elapsedMillis) {
        return new OpModeCommand(elapsedMillis, DriveToPose, MinTimeMillis, SettleTimeMillis, DriveSettleThresholdRatio,
                DrivePowerScale, IntakePower, IntakeHoldMillis, ShooterEnabled,
                ToroidShootRevolutions, ToroidTimeoutMillis,
                RequireActionStarted, RequireShooterEnabled, RequireShotQuality);
    }

    public static OpModeCommand driveDirectToPoseCommand(Pose2d pose) {
        return new OpModeCommand(
                null,
                pose,
                MIN_TIME_STANDARD,
                SETTLE_TIME_STANDARD,
                SETTLE_RATIO_STANDARD,
                null,
                null,
                null,
                null,
                null,
                null,
                false,
                false,
                false);
    }

    public static OpModeCommand driveDirectToPosePreciseCommand(Pose2d pose) {
        return new OpModeCommand(
                null,
                pose,
                MIN_TIME_STANDARD,
                SETTLE_TIME_STANDARD,
                PRECISE_SETTLE_RATIO,
                null,
                null,
                null,
                null,
                null,
                null,
                false,
                false,
                false);
    }

    public static OpModeCommand driveDirectToPoseFastCommand(Pose2d pose) {
        return new OpModeCommand(
                null,
                pose,
                MIN_TIME_NONE,
                SETTLE_TIME_NONE,
                SETTLE_RATIO_COARSE,
                null,
                null,
                null,
                null,
                null,
                null,
                false,
                false,
                false);
    }

    public static OpModeCommand driveDirectToPoseScaledCommand(Pose2d pose, double drivePowerScale) {
        double scale = Math.max(0.0, Math.min(1.0, drivePowerScale));
        return new OpModeCommand(
                null,
                pose,
                MIN_TIME_STANDARD,
                SETTLE_TIME_STANDARD,
                SETTLE_RATIO_STANDARD,
                scale,
                null,
                null,
                null,
                null,
                null,
                false,
                false,
                false);
    }

    public static OpModeCommand waitUntil(int millis) {
        return new OpModeCommand(
                millis,
                null,
                MIN_TIME_NONE,
                SETTLE_TIME_NONE,
                SETTLE_RATIO_STANDARD,
                null,
                null,
                null,
                null,
                null,
                null,
                false,
                false,
                false);
    }

    public static OpModeCommand waitCommand(Double waitMillis) {
        return new OpModeCommand(
                null,
                null,
                waitMillis,
                SETTLE_TIME_NONE,
                SETTLE_RATIO_STANDARD,
                null,
                null,
                null,
                null,
                null,
                null,
                false,
                false,
                false);
    }

    public static OpModeCommand shooterEnableCommand(boolean enabled) {
        return new OpModeCommand(
                null,
                null,
                MIN_TIME_STANDARD,
                SETTLE_TIME_NONE,
                SETTLE_RATIO_STANDARD,
                null,
                null,
                null,
                enabled,
                null,
                null,
                true,
                true,
                false);
    }

    public static OpModeCommand intakePowerCommand(double power) {
        return new OpModeCommand(
                null,
                null,
                MIN_TIME_NONE,
                SETTLE_TIME_NONE,
                SETTLE_RATIO_STANDARD,
                null,
                power,
                null,
                null,
                null,
                null,
                true,
                false,
                false);
    }

    public static OpModeCommand intakePowerHoldCommand(double power, double holdMillis) {
        return new OpModeCommand(
                null,
                null,
                MIN_TIME_NONE,
                SETTLE_TIME_NONE,
                SETTLE_RATIO_STANDARD,
                null,
                power,
                Math.max(0.0, holdMillis),
                null,
                null,
                null,
                true,
                false,
                false);
    }

    public static OpModeCommand shotQualityReadyCommand() {
        return new OpModeCommand(
                null,
                null,
                MIN_TIME_NONE,
                SHOT_QUALITY_SETTLE_TIME,
                SETTLE_RATIO_STANDARD,
                null,
                null,
                null,
                null,
                null,
                null,
                false,
                false,
                true);
    }

    public static OpModeCommand toroidShootStepsCommand(int steps, double timeoutMillis) {
        // 3 steps = 1 revolution (120 degrees per step).
        return toroidShootRevolutionsCommand(steps / 3.0, timeoutMillis);
    }

    public static OpModeCommand toroidShootRevolutionsCommand(double revolutions, double timeoutMillis) {
        return new OpModeCommand(
                null,
                null,
                MIN_TIME_NONE,
                SETTLE_TIME_NONE,
                SETTLE_RATIO_STANDARD,
                null,
                null,
                null,
                null,
                revolutions,
                timeoutMillis,
                true,
                false,
                false);
    }
}
