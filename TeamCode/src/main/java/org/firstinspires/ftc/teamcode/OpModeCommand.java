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

    public final Integer WaitUntilElapsedMillis;
    public final Pose2d DriveToPose;
    public final Double MinTimeMillis;
    public final Double SettleTimeMillis;
    public final Double DriveSettleThresholdRatio;
    public final Double DrivePowerScale;
    public final Double IntakePower;
    public final Boolean ShooterEnabled;
    public final Integer ToroidShootSteps;
    public final Double ToroidTimeoutMillis;
    public final boolean RequireActionStarted;
    public final boolean RequireShooterEnabled;

    private OpModeCommand(
            @Nullable Integer waitUntilElapsedMillis,
            @Nullable Pose2d driveDirectToPose,
            @NonNull Double minTimeMillis,
            @NonNull Double settleTimeMillis,
            @NonNull Double settleThresholdRatio,
            @Nullable Double drivePowerScale,
            @Nullable Double intakePower,
            @Nullable Boolean shooterEnabled,
            @Nullable Integer toroidShootSteps,
            @Nullable Double toroidTimeoutMillis,
            boolean requireActionStarted,
            boolean requireShooterEnabled) {
        WaitUntilElapsedMillis = waitUntilElapsedMillis;
        DriveToPose = driveDirectToPose;
        MinTimeMillis = minTimeMillis;
        SettleTimeMillis = settleTimeMillis;
        DriveSettleThresholdRatio = settleThresholdRatio;
        DrivePowerScale = drivePowerScale;
        IntakePower = intakePower;
        ShooterEnabled = shooterEnabled;
        ToroidShootSteps = toroidShootSteps;
        ToroidTimeoutMillis = toroidTimeoutMillis;
        RequireActionStarted = requireActionStarted;
        RequireShooterEnabled = requireShooterEnabled;
    }

    public OpModeCommand(
            @NonNull Integer waitUntilElapsedMillis,
            @Nullable Pose2d driveDirectToPose,
            @NonNull Double minTimeMillis,
            @NonNull Double settleTimeMillis,
            @NonNull Double settleThresholdRatio) {
        this(waitUntilElapsedMillis, driveDirectToPose, minTimeMillis, settleTimeMillis, settleThresholdRatio,
                null,
                null, null, null, null, false, false);
    }

    public OpModeCommand(
            @Nullable Pose2d driveDirectToPose,
            @NonNull Double minTimeMillis,
            @NonNull Double settleTimeMillis,
            @NonNull Double settleThresholdRatio) {
        this(0, driveDirectToPose, minTimeMillis, settleTimeMillis, settleThresholdRatio,
                null,
                null, null, null, null, false, false);
    }

    public OpModeCommand withWaitUntil(int elapsedMillis) {
        return new OpModeCommand(elapsedMillis, DriveToPose, MinTimeMillis, SettleTimeMillis, DriveSettleThresholdRatio,
                DrivePowerScale, IntakePower, ShooterEnabled, ToroidShootSteps, ToroidTimeoutMillis,
                RequireActionStarted, RequireShooterEnabled);
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
                enabled,
                null,
                null,
                true,
                true);
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
                true,
                false);
    }

    public static OpModeCommand toroidShootStepsCommand(int steps, double timeoutMillis) {
        return new OpModeCommand(
                null,
                null,
                MIN_TIME_NONE,
                SETTLE_TIME_NONE,
                SETTLE_RATIO_STANDARD,
                null,
                null,
                null,
                steps,
                timeoutMillis,
                true,
                false);
    }
}
