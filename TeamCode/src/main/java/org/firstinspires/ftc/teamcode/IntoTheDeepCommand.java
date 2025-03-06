package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.apache.commons.math3.analysis.function.Min;
import org.firstinspires.ftc.teamcode.util.AllianceColor;

import java.util.ArrayList;
import java.util.List;

public class IntoTheDeepCommand {

    private static final double MIN_TIME_STANDARD = 250;
    private static final double MIN_TIME_NONE = 0;
    private static final double SETTLE_TIME_STANDARD = 300;
    private static final double SETTLE_TIME_LONG = 600;
    private static final double SETTLE_TIME_NONE = 0;
    private static final double PRECISE_SETTLE_RATIO = 0.68d;
    private static final double SETTLE_RATIO_STANDARD = 1d;
    private static final double SETTLE_RATIO_COARSE = 3d;

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

    public static IntoTheDeepCommand driveToCollect(AllianceColor allianceColor, IntoTheDeepFieldPosition block, boolean pincerOpen, boolean armLowered) {
        IntoTheDeepCollectPosition collectPosition = IntoTheDeepPose.getBlockCollectPose(allianceColor, block, armLowered);
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

    public static IntoTheDeepCommand clipCollectApproach(AllianceColor allianceColor, boolean pincerOpen) {
        return new IntoTheDeepCommand(
                IntoTheDeepAppendageCommand.clipCollect(pincerOpen),
                IntoTheDeepPose.CLIP_COLLECT_APPROACH.getPose(allianceColor),
                MIN_TIME_STANDARD,
                SETTLE_TIME_STANDARD,
                SETTLE_RATIO_STANDARD);
    }

    public static IntoTheDeepCommand clipCollectCollect(AllianceColor allianceColor, boolean pincerOpen) {
        return new IntoTheDeepCommand(
                IntoTheDeepAppendageCommand.clipCollect(pincerOpen),
                IntoTheDeepPose.CLIP_COLLECT_COLLECT.getPose(allianceColor),
                MIN_TIME_STANDARD,
                pincerOpen ? SETTLE_TIME_LONG : SETTLE_TIME_STANDARD,
                SETTLE_RATIO_STANDARD);
    }

    public static IntoTheDeepCommand clipScoreApproach(AllianceColor allianceColor, int index) {
        int sign = allianceColor == AllianceColor.RED ? 1 : -1;
        Pose2d pose = IntoTheDeepPose.CLIP_SCORE_APPROACH.getPose(allianceColor)
                .plus(new Pose2d(sign * index * 1.5 /* 1.5 inches per scored block */, 0, 0));
        return new IntoTheDeepCommand(
                IntoTheDeepAppendageCommand.clipClipping(),
                pose,
                MIN_TIME_NONE,
                SETTLE_TIME_NONE,
                SETTLE_RATIO_COARSE);
    }

    public static IntoTheDeepCommand clipScoreRetreat(AllianceColor allianceColor, int index) {
        int sign = allianceColor == AllianceColor.RED ? 1 : -1;
        Pose2d pose = IntoTheDeepPose.CLIP_SCORE_APPROACH.getPose(allianceColor)
                .plus(new Pose2d(sign * index * 1.5 /* 1.5 inches per scored block */, 0, 0));
        return new IntoTheDeepCommand(
                IntoTheDeepAppendageCommand.clipAfterScore(),
                pose,
                MIN_TIME_NONE,
                SETTLE_TIME_NONE,
                SETTLE_RATIO_COARSE);
    }

    public static IntoTheDeepCommand clipScoreScore(AllianceColor allianceColor, int index) {
        int sign = allianceColor == AllianceColor.RED ? 1 : -1;
        Pose2d pose = IntoTheDeepPose.CLIP_SCORE_SCORE.getPose(allianceColor)
                .plus(new Pose2d(sign * index * 1.5 /* 1.5 inches per scored block */, 0, 0));
        return new IntoTheDeepCommand(
                IntoTheDeepAppendageCommand.clipClipping(),
                pose,
                MIN_TIME_STANDARD,
                SETTLE_TIME_STANDARD,
                SETTLE_RATIO_STANDARD);
    }

    public static IntoTheDeepCommand clipScorePostScore(AllianceColor allianceColor, int index) {
        int sign = allianceColor == AllianceColor.RED ? 1 : -1;
        Pose2d pose = IntoTheDeepPose.CLIP_SCORE_SCORE.getPose(allianceColor)
                .plus(new Pose2d(sign * index * 1.5 /* 1.5 inches per scored block */, 0, 0));
        return new IntoTheDeepCommand(
                IntoTheDeepAppendageCommand.clipAfterScore(),
                pose,
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

    public static IntoTheDeepCommand driveDirectToPoseTucked(Pose2d pose) {
        return new IntoTheDeepCommand(
                IntoTheDeepAppendageCommand.tucked(),
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

    public static IntoTheDeepCommand driveDirectToPoseFastCommandTucked(Pose2d pose) {
        return driveDirectToPoseFastCommand(pose, IntoTheDeepAppendageCommand.tucked());
    }

    public static IntoTheDeepCommand driveDirectToPoseFastCommand(Pose2d pose, IntoTheDeepAppendageCommand appendageCommand) {
        return new IntoTheDeepCommand(
                appendageCommand,
                pose,
                MIN_TIME_NONE,
                SETTLE_TIME_NONE,
                SETTLE_RATIO_COARSE);
    }

    public static IntoTheDeepCommand waitUntil(int millis) {
        return new IntoTheDeepCommand(
                millis,
                null,
                null,
                MIN_TIME_NONE,
                SETTLE_TIME_NONE,
                SETTLE_RATIO_STANDARD);
    }

    public static IntoTheDeepCommand waitCommand(Double waitMillis) {
        return new IntoTheDeepCommand(
                null,
                null,
                waitMillis,
                SETTLE_TIME_NONE,
                SETTLE_RATIO_STANDARD);
    }

    public static List<IntoTheDeepCommand> clipSequence(AllianceColor allianceColor, int index) {
        List<IntoTheDeepCommand> commands = new ArrayList<>();
        commands.add(IntoTheDeepCommand.clipScoreApproach(allianceColor, index));
        commands.add(IntoTheDeepCommand.clipScoreScore(allianceColor, index));
        commands.add(IntoTheDeepCommand.clipScorePostScore(allianceColor, index));
        commands.add(IntoTheDeepCommand.clipScoreRetreat(allianceColor, index));
        return commands;
    }

    public static List<IntoTheDeepCommand> collectAndClipSequence(AllianceColor allianceColor, int index) {
        List<IntoTheDeepCommand> commands = new ArrayList<>();
        commands.add(IntoTheDeepCommand.clipCollectApproach(allianceColor, true));
        commands.add(IntoTheDeepCommand.clipCollectCollect(allianceColor, true));
        commands.add(IntoTheDeepCommand.clipCollectCollect(allianceColor, false));
        commands.addAll(clipSequence(allianceColor, index));
        return commands;
    }

    public static List<IntoTheDeepCommand> observationPushSequence(AllianceColor allianceColor) {
        List<IntoTheDeepCommand> commands = new ArrayList<>();
        commands.add(IntoTheDeepCommand.driveDirectToPoseFastCommandTucked(IntoTheDeepPose.OBS_PUSH_STAGING.getPose(allianceColor)));
        commands.add(IntoTheDeepCommand.driveDirectToPoseFastCommandTucked(IntoTheDeepPose.OBS_PUSH_START_1.getPose(allianceColor)));
        commands.add(IntoTheDeepCommand.driveDirectToPoseFastCommandTucked(IntoTheDeepPose.OBS_PUSH_TARGET_1.getPose(allianceColor)));
        commands.add(IntoTheDeepCommand.driveDirectToPoseFastCommandTucked(IntoTheDeepPose.OBS_PUSH_STAGING.getPose(allianceColor)));
        commands.add(IntoTheDeepCommand.driveDirectToPoseFastCommandTucked(IntoTheDeepPose.OBS_PUSH_START_2.getPose(allianceColor)));
        commands.add(IntoTheDeepCommand.driveDirectToPoseFastCommandTucked(IntoTheDeepPose.OBS_PUSH_TARGET_2.getPose(allianceColor)));
        return commands;
    }

    public static List<IntoTheDeepCommand> observationPushSample1Sequence(AllianceColor allianceColor) {
        List<IntoTheDeepCommand> commands = new ArrayList<>();
        commands.add(IntoTheDeepCommand.driveDirectToPoseFastCommandTucked(IntoTheDeepPose.OBS_PUSH_STAGING.getPose(allianceColor)));
        commands.add(IntoTheDeepCommand.driveDirectToPoseFastCommandTucked(IntoTheDeepPose.OBS_PUSH_START_1.getPose(allianceColor)));
        commands.add(IntoTheDeepCommand.driveDirectToPoseFastCommandTucked(IntoTheDeepPose.OBS_PUSH_TARGET_1.getPose(allianceColor)));
        return commands;
    }



    public static List<IntoTheDeepCommand> dunkSequence(AllianceColor allianceColor) {
        List<IntoTheDeepCommand> commands = new ArrayList<>();
        commands.add(IntoTheDeepCommand.driveToNet(allianceColor));
        commands.add(IntoTheDeepCommand.dunkAtNetClosed(allianceColor));
        commands.add(IntoTheDeepCommand.dunkAtNetOpen(allianceColor));
        commands.add(IntoTheDeepCommand.defensive());
        return commands;
    }

    public static List<IntoTheDeepCommand> collectBlockSequence(AllianceColor allianceColor, IntoTheDeepFieldPosition block) {
        List<IntoTheDeepCommand> commands = new ArrayList<>();
        commands.add(driveToCollect(allianceColor, block, true, false));
        commands.add(driveToCollect(allianceColor, block, true, true));
        commands.add(driveToCollect(allianceColor, block, false, true));
        commands.add(driveToCollect(allianceColor, block, false, false));
        return commands;
    }

    public static List<IntoTheDeepCommand> collectAndDunkBlockSequence(AllianceColor allianceColor, IntoTheDeepFieldPosition block) {
        boolean isRed = allianceColor == AllianceColor.RED;
        int ifRedSign = isRed ? -1 : 1;
        double alignedHeading = Math.toRadians(90 * ifRedSign);
        double acrossHeading = Math.toRadians(90 + (90 * ifRedSign));

        Vector2d block1Pos = IntoTheDeepFieldPosition.AUTON_PREP_BLOCK_NEUTRAL_1.getPosition(allianceColor);
        Vector2d block2Pos = IntoTheDeepFieldPosition.AUTON_PREP_BLOCK_NEUTRAL_2.getPosition(allianceColor);
        Vector2d block3Pos = IntoTheDeepFieldPosition.AUTON_PREP_BLOCK_NEUTRAL_3.getPosition(allianceColor);
        Pose2d block1PrepPose = new Pose2d(block1Pos.getX(), block1Pos.getY(), alignedHeading);
        Pose2d block2PrepPose = new Pose2d(block2Pos.getX(), block2Pos.getY(), alignedHeading);
        Pose2d block3PrepPose = new Pose2d(block3Pos.getX(), block3Pos.getY(), acrossHeading);

        List<IntoTheDeepCommand> commands = new ArrayList<>();
        if (block == IntoTheDeepFieldPosition.AUTON_BLOCK_NEUTRAL_1) {
            commands.add(driveDirectToPoseCommand(block1PrepPose));
        } else if (block == IntoTheDeepFieldPosition.AUTON_BLOCK_NEUTRAL_2) {
            commands.add(driveDirectToPoseCommand(block2PrepPose));
        } else if (block == IntoTheDeepFieldPosition.AUTON_BLOCK_NEUTRAL_3) {
            commands.add(driveDirectToPoseCommand(block3PrepPose));
        }
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

    public static IntoTheDeepCommand tucked() {
        return new IntoTheDeepCommand(
                0,
                IntoTheDeepAppendageCommand.tucked(),
                null,
                MIN_TIME_STANDARD,
                SETTLE_TIME_STANDARD,
                SETTLE_RATIO_STANDARD
        );
    }
}
