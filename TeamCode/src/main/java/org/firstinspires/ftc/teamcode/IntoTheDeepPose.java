package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

public enum IntoTheDeepPose {

    // Starting poses
    START_OBSERVATION_ZONE(IntoTheDeepFieldPosition.START_OBSERVATION_ZONE, Math.PI / 2.0),
    START_MID(IntoTheDeepFieldPosition.START_MID, Math.PI / 2.0),
    START_NET_ZONE(IntoTheDeepFieldPosition.START_NET_ZONE, Math.PI / 2.0),

    // Waypoints
    NET_PUSH_STAGING(IntoTheDeepFieldPosition.NET_PUSH_STAGING, Math.PI / 2.0),
    NET_PUSH_START_1(IntoTheDeepFieldPosition.NET_PUSH_START_1, Math.PI / 2.0),
    NET_PUSH_START_2(IntoTheDeepFieldPosition.NET_PUSH_START_2, Math.PI / 2.0),
    NET_PUSH_TARGET_1(IntoTheDeepFieldPosition.NET_PUSH_TARGET, Math.toRadians(90 - 18.43)),
    NET_PUSH_TARGET_2(IntoTheDeepFieldPosition.NET_PUSH_TARGET, Math.toRadians(90)),
    NET_PUSH_START_3_1(IntoTheDeepFieldPosition.NET_PUSH_START_3_1, Math.toRadians(90)),
    NET_PUSH_START_3_2(IntoTheDeepFieldPosition.NET_PUSH_START_3_2, Math.toRadians(90)),
    NET_PUSH_TARGET_3(IntoTheDeepFieldPosition.NET_PUSH_TARGET_3, Math.toRadians(90)),

    OBS_PUSH_STAGING(IntoTheDeepFieldPosition.OBS_PUSH_STAGING, Math.PI / 2.0),
    OBS_PUSH_START_1(IntoTheDeepFieldPosition.OBS_PUSH_START_1, Math.PI / 2.0),
    OBS_PUSH_START_2(IntoTheDeepFieldPosition.OBS_PUSH_START_2, Math.PI / 2.0),
    OBS_PUSH_TARGET_1(IntoTheDeepFieldPosition.OBS_PUSH_TARGET, Math.toRadians(90 + 18.43)),
    OBS_PUSH_TARGET_2(IntoTheDeepFieldPosition.OBS_PUSH_TARGET, Math.toRadians(90)),

    PARK_TARGET_FIRST(IntoTheDeepFieldPosition.PARK_TARGET_FIRST, Math.toRadians(90)),
    PARK_TARGET_SECOND(IntoTheDeepFieldPosition.PARK_TARGET_SECOND, Math.toRadians(90)),
    PARK_TARGET_OUT_OF_WAY_OBS(IntoTheDeepFieldPosition.PARK_TARGET_OUT_OF_WAY_OBS, 0),
    PARK_TARGET_OUT_OF_WAY_NET(IntoTheDeepFieldPosition.PARK_TARGET_OUT_OF_WAY_NET, 0),
    OBS_WAYPOINT(IntoTheDeepFieldPosition.OBS_WAYPOINT, 0),
    MID_WAYPOINT(IntoTheDeepFieldPosition.MID_WAYPOINT, 0),
    NET_WAYPOINT(IntoTheDeepFieldPosition.NET_WAYPOINT, Math.toRadians(45)),
    CLIP1_SCORE(IntoTheDeepFieldPosition.CLIP1_SCORE,Math.PI/2.0),

    // Teleop poses
    SUBMERSIBLE_APPROACH_ALLIANCE_SIDE(IntoTheDeepFieldPosition.SUBMERSIBLE_APPROACH_ALLIANCE_SIDE, Math.toRadians(90)),
    SUBMERSIBLE_APPROACH_OPPONENT_SIDE(IntoTheDeepFieldPosition.SUBMERSIBLE_APPROACH_OPPONENT_SIDE, Math.toRadians(270)),
    SUBMERSIBLE_APPROACH_REAR_SIDE(IntoTheDeepFieldPosition.SUBMERSIBLE_APPROACH_REAR_SIDE, Math.toRadians(0)),
    SUBMERSIBLE_APPROACH_AUDIENCE_SIDE(IntoTheDeepFieldPosition.SUBMERSIBLE_APPROACH_AUDIENCE_SIDE, Math.toRadians(180)),

    BASKET_APPROACH(IntoTheDeepFieldPosition.BASKET_APPROACH_FAST, Math.toRadians(45)),
    HIGH_BASKET_SCORING_APPROACH(IntoTheDeepFieldPosition.HIGH_BASKET_SCORING_APPROACH, Math.toRadians(45)),

    CLIP_SCORE_APPROACH(IntoTheDeepFieldPosition.AUTON_CLIP_SCORE_APPROACH, Math.toRadians(90)),
    CLIP_SCORE_SCORE(IntoTheDeepFieldPosition.AUTON_CLIP_SCORE_SCORE, Math.toRadians(90)),
    CLIP_COLLECT_APPROACH(IntoTheDeepFieldPosition.AUTON_CLIP_COLLECT_APPROACH, Math.toRadians(270)),
    CLIP_COLLECT_COLLECT(IntoTheDeepFieldPosition.AUTON_CLIP_COLLECT_COLLECT, Math.toRadians(270));

    public final Pose2d BluePose;
    public final Pose2d RedPose;

    IntoTheDeepPose(IntoTheDeepFieldPosition fieldPosition, double headingBlue) {
        BluePose = new Pose2d(fieldPosition.BluePosition, headingBlue);
        RedPose = new Pose2d(fieldPosition.RedPosition, headingBlue + Math.PI);
    }

    public Pose2d getPose(AllianceColor allianceColor) {
        switch (allianceColor) {
            case RED:
                return RedPose;
            case BLUE:
                return BluePose;
            default:
                return new Pose2d();
        }
    }

    public static IntoTheDeepCollectPosition getBlockCollectPose(AllianceColor allianceColor, IntoTheDeepFieldPosition block, boolean armLowered) {
        // These are the cases that work for the blocks we collect
        boolean isRed = allianceColor == AllianceColor.RED;
        int ifRedSign = isRed ? -1 : 1;
        double alignedHeading = Math.toRadians(90 * ifRedSign);
        double acrossHeading = Math.toRadians(90 + (90 * ifRedSign));

        Vector2d blockPosition = block.getPosition(allianceColor);
        Pose2d collectPose = new Pose2d();
        double wristSignal = TerabytesIntoTheDeep.AUTON_COLLECT_WRIST_SIGNAL_ALIGNED;
        double collectOffsetExtension = TerabytesIntoTheDeep.AUTON_COLLECT_X_OFFSET_DISTANCE;
        double collectOffsetLateral = TerabytesIntoTheDeep.AUTON_COLLECT_Y_OFFSET_DISTANCE;
        switch (block) {
            case AUTON_BLOCK_NEUTRAL_1:
            case AUTON_BLOCK_NEUTRAL_2:
                collectPose = new Pose2d(
                        blockPosition.getX() + (collectOffsetLateral * ifRedSign),
                        blockPosition.getY() + (collectOffsetExtension * ifRedSign),
                        alignedHeading);
                break;
            case AUTON_BLOCK_NEUTRAL_3:
                collectPose = new Pose2d(
                        blockPosition.getX() + (collectOffsetExtension * ifRedSign),
                        blockPosition.getY() + (collectOffsetLateral * ifRedSign),
                        acrossHeading);
                wristSignal = TerabytesIntoTheDeep.AUTON_COLLECT_WRIST_SIGNAL_ACROSS;
                break;
            default:
                break;
        }

        return new IntoTheDeepCollectPosition(
                collectPose,
                armLowered ? TerabytesIntoTheDeep.AUTON_COLLECT_HEIGHT_SIGNAL : TerabytesIntoTheDeep.AUTON_PRE_COLLECT_HEIGHT_SIGNAL,
                TerabytesIntoTheDeep.AUTON_COLLECT_DISTANCE_SIGNAL,
                wristSignal);
    }
}
