package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

import java.util.ArrayList;
import java.util.List;

public class IntoTheDeepCommand {

    private static final double MIN_TIME_STANDARD = 250;
    private static final double MIN_TIME_NONE = 0;
    private static final double SETTLE_TIME_STANDARD = 250;
    private static final double SETTLE_TIME_NONE = 0;
    private static final double PRECISE_SETTLE_RATIO = 0.68d;
    private static final double SETTLE_RATIO_STANDARD = 1d;
    private static final double SETTLE_RATIO_COARSE = 5d;

    public final IntoTheDeepAppendageCommand AppendageCommand;
    public final Pose2d DriveToPose;
    public final Double MinTimeMillis;
    public final Double SettleTimeMillis;
    public final Double DriveSettleThresholdRatio;

    public IntoTheDeepCommand(
            @Nullable IntoTheDeepAppendageCommand appendageCommand,
            @Nullable Pose2d driveDirectToPose,
            @NonNull Double minTimeMillis,
            @NonNull Double settleTimeMillis,
            @NonNull Double settleThresholdRatio) {
        AppendageCommand = appendageCommand;
        DriveToPose = driveDirectToPose;
        MinTimeMillis = minTimeMillis;
        SettleTimeMillis = settleTimeMillis;
        DriveSettleThresholdRatio = settleThresholdRatio;
    }

    public static IntoTheDeepCommand driveToNet(AllianceColor allianceColor) {
        return new IntoTheDeepCommand(
                IntoTheDeepAppendageCommand.highBasketPre(),
                IntoTheDeepPose.HIGH_BASKET_SCORING_APPROACH.getPose(allianceColor),
                MIN_TIME_STANDARD,
                SETTLE_TIME_STANDARD,
                SETTLE_RATIO_STANDARD);
    }

    public static IntoTheDeepCommand dunkAtNetClosed(AllianceColor allianceColor) {
        return new IntoTheDeepCommand(
                IntoTheDeepAppendageCommand.highBasketDunk(),
                IntoTheDeepPose.HIGH_BASKET_SCORING_APPROACH.getPose(allianceColor),
                MIN_TIME_STANDARD,
                SETTLE_TIME_STANDARD,
                SETTLE_RATIO_STANDARD);
    }

    public static IntoTheDeepCommand dunkAtNetOpen(AllianceColor allianceColor) {
        return new IntoTheDeepCommand(
                IntoTheDeepAppendageCommand.highBasketDunkRelease(),
                IntoTheDeepPose.HIGH_BASKET_SCORING_APPROACH.getPose(allianceColor),
                MIN_TIME_STANDARD,
                SETTLE_TIME_STANDARD,
                SETTLE_RATIO_STANDARD);
    }

    public static IntoTheDeepCommand driveDirectToPoseCommand(Pose2d pose) {
        return new IntoTheDeepCommand(
                null,
                pose,
                MIN_TIME_STANDARD,
                SETTLE_TIME_STANDARD,
                SETTLE_RATIO_STANDARD);
    }

    public static IntoTheDeepCommand driveDirectToPoseDefensive(Pose2d pose) {
        return new IntoTheDeepCommand(
                IntoTheDeepAppendageCommand.defensive(),
                pose,
                MIN_TIME_STANDARD,
                SETTLE_TIME_STANDARD,
                SETTLE_RATIO_STANDARD);
    }

    public static IntoTheDeepCommand driveDirectToPoseFastCommand(Pose2d pose) {
        return new IntoTheDeepCommand(
                null,
                pose,
                MIN_TIME_NONE,
                SETTLE_TIME_NONE,
                SETTLE_RATIO_COARSE);
    }

    public static IntoTheDeepCommand waitCommand(Double waitMillis) {
        return new IntoTheDeepCommand(
                null,
                null,
                waitMillis,
                SETTLE_TIME_NONE,
                SETTLE_RATIO_STANDARD);
    }

    public static List<IntoTheDeepCommand> dunkSequence(AllianceColor allianceColor) {
        List<IntoTheDeepCommand> commands = new ArrayList<>();
        commands.add(IntoTheDeepCommand.driveToNet(allianceColor));
        commands.add(IntoTheDeepCommand.dunkAtNetClosed(allianceColor));
        commands.add(IntoTheDeepCommand.dunkAtNetOpen(allianceColor));
        return commands;
    }
}
