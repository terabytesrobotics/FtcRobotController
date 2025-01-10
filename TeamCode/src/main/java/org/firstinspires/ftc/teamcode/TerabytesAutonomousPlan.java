package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

import java.util.ArrayList;
import java.util.List;

public enum TerabytesAutonomousPlan {
    START_OBS_WAIT_SCORE_WAIT,
    START_NET_SCORE_COLLECT_SCORE_PARK,
    START_NET_SCORE_COLLECT_SCORE_WAIT;

    private static double OBS_WAIT_DURATION_SECONDS = 17.25;

    public List<IntoTheDeepCommand> getCommandSequence(AllianceColor color) {
        ArrayList<IntoTheDeepCommand> commands = new ArrayList<>();

        Pose2d parkOutOfWayObs = IntoTheDeepPose.PARK_TARGET_OUT_OF_WAY_OBS.getPose(color);
        Pose2d parkOutOfWayNet = IntoTheDeepPose.PARK_TARGET_OUT_OF_WAY_NET.getPose(color);

        switch (this) {
            case START_OBS_WAIT_SCORE_WAIT:
                commands.add(IntoTheDeepCommand.driveDirectToPoseFastCommand(parkOutOfWayObs));
                commands.add(IntoTheDeepCommand.waitUntil((int)(OBS_WAIT_DURATION_SECONDS * 1000)));
                commands.add(IntoTheDeepCommand.driveDirectToPoseFastCommand(IntoTheDeepPose.OBS_WAYPOINT.getPose(color)));
                commands.add(IntoTheDeepCommand.driveDirectToPoseFastCommand(IntoTheDeepPose.MID_WAYPOINT.getPose(color)));
                commands.addAll(IntoTheDeepCommand.dunkSequence(color));
                break;
            case START_NET_SCORE_COLLECT_SCORE_WAIT:
            case START_NET_SCORE_COLLECT_SCORE_PARK:
                commands.addAll(IntoTheDeepCommand.dunkSequence(color));
                commands.addAll(IntoTheDeepCommand.collectAndDunkBlockSequence(color, IntoTheDeepFieldPosition.AUTON_BLOCK_NEUTRAL_1));
                commands.addAll(IntoTheDeepCommand.collectAndDunkBlockSequence(color, IntoTheDeepFieldPosition.AUTON_BLOCK_NEUTRAL_2));
                break;
        }

        Pose2d parkTargetFirst = IntoTheDeepPose.PARK_TARGET_FIRST.getPose(color);
        Pose2d parkPose = IntoTheDeepPose.PARK_TARGET_SECOND.getPose(color);
        switch (this) {
            case START_NET_SCORE_COLLECT_SCORE_WAIT:
            case START_OBS_WAIT_SCORE_WAIT:
                parkPose = IntoTheDeepPose.PARK_TARGET_OUT_OF_WAY_NET.getPose(color);
                break;
            case START_NET_SCORE_COLLECT_SCORE_PARK:
            default:
                break;
        }

        commands.add(IntoTheDeepCommand.driveDirectToPoseCommand(parkPose));

        return commands;
    }

    public Pose2d getStartingPose(AllianceColor allianceColor) {
        switch (this) {
            case START_OBS_WAIT_SCORE_WAIT:
                return IntoTheDeepPose.START_OBSERVATION_ZONE.getPose(allianceColor);
            case START_NET_SCORE_COLLECT_SCORE_PARK:
            default:
                return IntoTheDeepPose.START_NET_ZONE.getPose(allianceColor);
        }
    }
}
