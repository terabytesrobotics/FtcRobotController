package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class OpModeCommand {

    private static final double MIN_TIME_STANDARD = 250;
    private static final double MIN_TIME_NONE = 0;
    private static final double SETTLE_TIME_STANDARD = 300;
    private static final double SETTLE_TIME_LONG = 900;
    private static final double SETTLE_TIME_NONE = 0;
    private static final double PRECISE_SETTLE_RATIO = 0.68d;
    private static final double SETTLE_RATIO_STANDARD = 1d;
    private static final double SETTLE_RATIO_COARSE = 3d;

    public final Integer WaitUntilElapsedMillis;
    public final Pose2d DriveToPose;
    public final Double MinTimeMillis;
    public final Double SettleTimeMillis;
    public final Double DriveSettleThresholdRatio;
    public final Integer SpindexerTargetSlot;
    public final Integer SpindexerDeltaSlots;
    public final Double IntakePower;
    public final Boolean Kick;
    public final Boolean ShooterEnabled;
    public final boolean RequireSpindexerSettled;
    public final boolean RequireKickerIdle;
    public final boolean RequireActionStarted;
    public final boolean RequireShooterEnabled;

    private OpModeCommand(
            @Nullable Integer waitUntilElapsedMillis,
            @Nullable Pose2d driveDirectToPose,
            @NonNull Double minTimeMillis,
            @NonNull Double settleTimeMillis,
            @NonNull Double settleThresholdRatio,
            @Nullable Integer spindexerTargetSlot,
            @Nullable Integer spindexerDeltaSlots,
            @Nullable Double intakePower,
            @Nullable Boolean kick,
            @Nullable Boolean shooterEnabled,
            boolean requireSpindexerSettled,
            boolean requireKickerIdle,
            boolean requireActionStarted,
            boolean requireShooterEnabled) {
        WaitUntilElapsedMillis = waitUntilElapsedMillis;
        DriveToPose = driveDirectToPose;
        MinTimeMillis = minTimeMillis;
        SettleTimeMillis = settleTimeMillis;
        DriveSettleThresholdRatio = settleThresholdRatio;
        SpindexerTargetSlot = spindexerTargetSlot;
        SpindexerDeltaSlots = spindexerDeltaSlots;
        IntakePower = intakePower;
        Kick = kick;
        ShooterEnabled = shooterEnabled;
        RequireSpindexerSettled = requireSpindexerSettled;
        RequireKickerIdle = requireKickerIdle;
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
                null, null, null, null, null, false, false, false, false);
    }

    public OpModeCommand(
            @Nullable Pose2d driveDirectToPose,
            @NonNull Double minTimeMillis,
            @NonNull Double settleTimeMillis,
            @NonNull Double settleThresholdRatio) {
        this(0, driveDirectToPose, minTimeMillis, settleTimeMillis, settleThresholdRatio,
                null, null, null, null, null, false, false, false, false);
    }

    public OpModeCommand withWaitUntil(int elapsedMillis) {
        return new OpModeCommand(elapsedMillis, DriveToPose, MinTimeMillis, SettleTimeMillis, DriveSettleThresholdRatio,
                SpindexerTargetSlot, SpindexerDeltaSlots, IntakePower, Kick, ShooterEnabled,
                RequireSpindexerSettled, RequireKickerIdle, RequireActionStarted, RequireShooterEnabled);
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
                false,
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
                false,
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
                false,
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
                false,
                false,
                false,
                false);
    }

    public static OpModeCommand spindexerToSlotCommand(int slot) {
        return new OpModeCommand(
                null,
                null,
                MIN_TIME_STANDARD,
                SETTLE_TIME_LONG,
                PRECISE_SETTLE_RATIO,
                slot,
                null,
                null,
                null,
                null,
                true,
                false,
                true,
                false);
    }

    public static OpModeCommand advanceSpindexerCommand(int deltaSlots) {
        return new OpModeCommand(
                null,
                null,
                MIN_TIME_STANDARD,
                SETTLE_TIME_LONG,
                PRECISE_SETTLE_RATIO,
                null,
                deltaSlots,
                null,
                null,
                null,
                true,
                false,
                true,
                false);
    }

    public static OpModeCommand kickCommand() {
        return new OpModeCommand(
                null,
                null,
                MIN_TIME_STANDARD,
                SETTLE_TIME_LONG,
                SETTLE_RATIO_STANDARD,
                null,
                null,
                null,
                true,
                null,
                true,
                true,
                true,
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
                null,
                enabled,
                false,
                false,
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
                null,
                power,
                null,
                null,
                false,
                false,
                true,
                false);
    }
}
