package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

import java.util.ArrayList;
import java.util.List;

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

    public OpModeCommand(
            @NonNull Integer waitUntilElapsedMillis,
            @Nullable Pose2d driveDirectToPose,
            @NonNull Double minTimeMillis,
            @NonNull Double settleTimeMillis,
            @NonNull Double settleThresholdRatio) {
        WaitUntilElapsedMillis = waitUntilElapsedMillis;
        DriveToPose = driveDirectToPose;
        MinTimeMillis = minTimeMillis;
        SettleTimeMillis = settleTimeMillis;
        DriveSettleThresholdRatio = settleThresholdRatio;
    }

    public OpModeCommand(
            @Nullable Pose2d driveDirectToPose,
            @NonNull Double minTimeMillis,
            @NonNull Double settleTimeMillis,
            @NonNull Double settleThresholdRatio) {
        this(0, driveDirectToPose, minTimeMillis, settleTimeMillis, settleThresholdRatio);
    }

    public OpModeCommand withWaitUntil(int elapsedMillis) {
        return new OpModeCommand(elapsedMillis, DriveToPose, MinTimeMillis, SettleTimeMillis, SettleTimeMillis);
    }

    public static OpModeCommand driveDirectToPoseCommand(Pose2d pose) {
        return new OpModeCommand(
                null,
                pose,
                MIN_TIME_STANDARD,
                SETTLE_TIME_STANDARD,
                SETTLE_RATIO_STANDARD);
    }

    public static OpModeCommand driveDirectToPoseFastCommand(Pose2d pose) {
        return new OpModeCommand(
                null,
                pose,
                MIN_TIME_NONE,
                SETTLE_TIME_NONE,
                SETTLE_RATIO_COARSE);
    }

    public static OpModeCommand waitUntil(int millis) {
        return new OpModeCommand(
                millis,
                null,
                MIN_TIME_NONE,
                SETTLE_TIME_NONE,
                SETTLE_RATIO_STANDARD);
    }

    public static OpModeCommand waitCommand(Double waitMillis) {
        return new OpModeCommand(
                null,
                null,
                waitMillis,
                SETTLE_TIME_NONE,
                SETTLE_RATIO_STANDARD);
    }
}
