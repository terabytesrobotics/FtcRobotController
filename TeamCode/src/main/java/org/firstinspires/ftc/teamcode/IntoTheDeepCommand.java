package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.apache.commons.math3.analysis.function.Min;
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

    public final Integer WaitUntilElapsedMillis;
    public final IntoTheDeepAppendageCommand AppendageCommand;
    public final Pose2d DriveToPose;
    public final Double MinTimeMillis;
    public final Double SettleTimeMillis;
    public final Double DriveSettleThresholdRatio;

    public IntoTheDeepCommand(
            @NonNull Integer waitUntilElapsedMillis,
            @Nullable IntoTheDeepAppendageCommand appendageCommand,
            @Nullable Pose2d driveDirectToPose,
            @NonNull Double minTimeMillis,
            @NonNull Double settleTimeMillis,
            @NonNull Double settleThresholdRatio) {
        WaitUntilElapsedMillis = waitUntilElapsedMillis;
        AppendageCommand = appendageCommand;
        DriveToPose = driveDirectToPose;
        MinTimeMillis = minTimeMillis;
        SettleTimeMillis = settleTimeMillis;
        DriveSettleThresholdRatio = settleThresholdRatio;
    }

    public IntoTheDeepCommand(
            @Nullable IntoTheDeepAppendageCommand appendageCommand,
            @Nullable Pose2d driveDirectToPose,
            @NonNull Double minTimeMillis,
            @NonNull Double settleTimeMillis,
            @NonNull Double settleThresholdRatio) {
        this(0, appendageCommand, driveDirectToPose, minTimeMillis, settleTimeMillis, settleThresholdRatio);
    }

    public IntoTheDeepCommand withWaitUntil(int elapsedMillis) {
        return new IntoTheDeepCommand(elapsedMillis, AppendageCommand, DriveToPose, MinTimeMillis, SettleTimeMillis, SettleTimeMillis);
    }

    public static IntoTheDeepCommand driveToCollect(AllianceColor allianceColor, IntoTheDeepFieldPosition block, boolean pincerOpen) {
        IntoTheDeepCollectPosition collectPosition = IntoTheDeepPose.getBlockCollectPose(allianceColor, block);
        IntoTheDeepAppendageCommand appendageCommand = pincerOpen ?
                IntoTheDeepAppendageCommand.collectingOpen(collectPosition.DistanceSignal, collectPosition.HeightSignal, collectPosition.WristSignal) :
                IntoTheDeepAppendageCommand.collectingClosed(collectPosition.DistanceSignal, collectPosition.HeightSignal, collectPosition.WristSignal);

        return new IntoTheDeepCommand(
                appendageCommand,
                collectPosition.CollectPose,
                MIN_TIME_STANDARD,
                SETTLE_TIME_STANDARD,
                SETTLE_RATIO_STANDARD);
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

    public static List<IntoTheDeepCommand> collectBlockSequence(AllianceColor allianceColor, IntoTheDeepFieldPosition block) {
        List<IntoTheDeepCommand> commands = new ArrayList<>();
        commands.add(driveToCollect(allianceColor, block, true));
        commands.add(driveToCollect(allianceColor, block, false));
        return commands;
    }

    public static List<IntoTheDeepCommand> collectAndDunkBlockSequence(AllianceColor allianceColor, IntoTheDeepFieldPosition block) {
        List<IntoTheDeepCommand> commands = new ArrayList<>();
        commands.addAll(collectBlockSequence(allianceColor, block));
        commands.addAll(dunkSequence(allianceColor));
        return commands;
    }

    public static IntoTheDeepCommand defensive() {
        return new IntoTheDeepCommand(
                0,
                IntoTheDeepAppendageCommand.defensive(),
                null,
                MIN_TIME_STANDARD,
                SETTLE_TIME_STANDARD,
                SETTLE_RATIO_STANDARD
        );
    }
}
