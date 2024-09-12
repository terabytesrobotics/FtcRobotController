package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.util.BlueGrabberState;
import org.firstinspires.ftc.teamcode.util.CollectorState;
import org.firstinspires.ftc.teamcode.util.GreenGrabberState;

public class TerabytesCommand {
    public final Pose2d DriveDirectToPose;
    public final Double MinTimeMillis;
    public final Double SettleTimeMillis;
    public final Double SettleThresholdRatio;
    public final Boolean FrontCameraOn;

    public TerabytesCommand(
            @Nullable Boolean frontCameraOn,
            @Nullable Pose2d driveDirectToPose,
            @NonNull Double minTimeMillis,
            @NonNull Double settleTimeMillis,
            @NonNull Double settleThresholdRatio) {
        FrontCameraOn = frontCameraOn;
        DriveDirectToPose = driveDirectToPose;
        MinTimeMillis = minTimeMillis;
        SettleTimeMillis = settleTimeMillis;
        SettleThresholdRatio = settleThresholdRatio;
    }

    public static TerabytesCommand driveDirectToPoseCommand(Pose2d pose) {
        return new TerabytesCommand(
                null,
                pose,
                250d,
                250d,
                1d);
    }

    public static TerabytesCommand driveDirectToPoseFastCommand(Pose2d pose) {
        return new TerabytesCommand(
                null,
                pose,
                0d,
                0d,
                5d);
    }

    public static TerabytesCommand waitCommand(Double waitMillis) {
        return new TerabytesCommand(
                null,
                null,
                waitMillis,
                0d,
                1d);
    }

    public static TerabytesCommand turnOnFrontCamera() {
        return new TerabytesCommand(
                true,
                null,
                1000d,
                0d,
                1d);
    }
}
