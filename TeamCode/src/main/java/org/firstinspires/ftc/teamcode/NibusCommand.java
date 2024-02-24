package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.util.BlueGrabberState;
import org.firstinspires.ftc.teamcode.util.CollectorState;
import org.firstinspires.ftc.teamcode.util.GreenGrabberState;

public class NibusCommand {
    public final Double ScoringHeight;
    public final CollectorState CollectorState;
    public final BlueGrabberState BlueGrabberState;
    public final GreenGrabberState GreenGrabberState;
    public final Pose2d DriveDirectToPose;
    public final Double MinTimeMillis;
    public final Double SettleTimeMillis;
    public final Double SettleThresholdRatio;
    public final Boolean FrontCameraOn;

    public NibusCommand(
            @Nullable Boolean frontCameraOn,
            @Nullable Double scoringHeight,
            @Nullable CollectorState collectorState,
            @Nullable BlueGrabberState blueGrabberState,
            @Nullable GreenGrabberState greenGrabberState,
            @Nullable Pose2d driveDirectToPose,
            @NonNull Double minTimeMillis,
            @NonNull Double settleTimeMillis,
            @NonNull Double settleThresholdRatio) {
        FrontCameraOn = frontCameraOn;
        ScoringHeight = scoringHeight;
        CollectorState = collectorState;
        BlueGrabberState = blueGrabberState;
        GreenGrabberState = greenGrabberState;
        DriveDirectToPose = driveDirectToPose;
        MinTimeMillis = minTimeMillis;
        SettleTimeMillis = settleTimeMillis;
        SettleThresholdRatio = settleThresholdRatio;
    }

    public static NibusCommand collectorStateCommand(CollectorState collectorState) {
        return new NibusCommand(
                null,
                null,
                collectorState,
                null,
                null,
                null,
                250d,
                250d,
                1d);
    }

    public static NibusCommand driveDirectToPoseCommand(Pose2d pose) {
        return new NibusCommand(
                null,
                null,
                null,
                null,
                null,
                pose,
                250d,
                250d,
                1d);
    }

    public static NibusCommand driveDirectToPoseFastCommand(Pose2d pose) {
        return new NibusCommand(
                null,
                null,
                null,
                null,
                null,
                pose,
                0d,
                0d,
                5d);
    }

    public static NibusCommand driveDirectToPoseWithCollectorStateCommand(Pose2d pose, CollectorState collectorState) {
        return new NibusCommand(
                null,
                null,
                collectorState,
                null,
                null,
                pose,
                250d,
                250d,
                1d);
    }

    public static NibusCommand driveDirectToPoseWithCollectorStatePreciseCommand(Pose2d pose, CollectorState collectorState) {
        return new NibusCommand(
                null,
                null,
                collectorState,
                null,
                null,
                pose,
                250d,
                250d,
                0.75d);
    }

    public static NibusCommand scoringHeightCommand(Double scoringHeight) {
        return new NibusCommand(
                null,
                scoringHeight,
                null,
                null,
                null,
                null,
                250d,
                250d,
                1d);
    }

    public static NibusCommand grabberStateCommand(BlueGrabberState blueGrabberState, GreenGrabberState greenGrabberState) {
        return new NibusCommand(
                null,
                null,
                null,
                blueGrabberState,
                greenGrabberState,
                null,
                500d,
                0d,
                1d);
    }

    public static NibusCommand grabberStateCollectorStateCommand(BlueGrabberState blueGrabberState, GreenGrabberState greenGrabberState, CollectorState collectorState) {
        return new NibusCommand(
                null,
                null,
                collectorState,
                blueGrabberState,
                greenGrabberState,
                null,
                500d,
                0d,
                1d);
    }

    public static NibusCommand waitCommand(Double waitMillis) {
        return new NibusCommand(
                null,
                null,
                null,
                null,
                null,
                null,
                waitMillis,
                0d,
                1d);
    }

    public static NibusCommand turnOnFrontCamera() {
        return new NibusCommand(
                true,
                null,
                null,
                null,
                null,
                null,
                1000d,
                0d,
                1d);
    }
}
