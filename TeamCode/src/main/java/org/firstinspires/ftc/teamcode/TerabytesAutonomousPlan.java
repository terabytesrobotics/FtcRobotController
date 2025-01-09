package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

import java.util.ArrayList;
import java.util.List;

public enum TerabytesAutonomousPlan {
    START_NET_PUSH_NET_PARK,
    START_OBS_PUSH_NET_PARK,
    START_NET_SCORE_HIGH_PUSH_PARK,
    START_NET_SCORE_HIGH_COLLECT_SCORE_PARK;

    public List<IntoTheDeepCommand> getCommandSequence(AllianceColor color) {
        ArrayList<IntoTheDeepCommand> commands = new ArrayList<>();

        switch (this) {
            case START_OBS_PUSH_NET_PARK:
            case START_NET_PUSH_NET_PARK:
                commands.add(IntoTheDeepCommand.driveDirectToPoseCommand(IntoTheDeepPose.NET_PUSH_STAGING.getPose(color)));
                commands.add(IntoTheDeepCommand.driveDirectToPoseFastCommand(IntoTheDeepPose.NET_PUSH_START_1.getPose(color)));
                commands.add(IntoTheDeepCommand.driveDirectToPoseFastCommand(IntoTheDeepPose.NET_PUSH_TARGET_1.getPose(color)));
                commands.add(IntoTheDeepCommand.driveDirectToPoseFastCommand(IntoTheDeepPose.NET_PUSH_START_1.getPose(color)));
                commands.add(IntoTheDeepCommand.driveDirectToPoseFastCommand(IntoTheDeepPose.NET_PUSH_START_2.getPose(color)));
                commands.add(IntoTheDeepCommand.driveDirectToPoseFastCommand(IntoTheDeepPose.NET_PUSH_TARGET_2.getPose(color)));
                break;
            case START_NET_SCORE_HIGH_PUSH_PARK:
                commands.addAll(IntoTheDeepCommand.dunkSequence(color));
                commands.add(IntoTheDeepCommand.driveDirectToPoseDefensive(IntoTheDeepPose.NET_PUSH_STAGING.getPose(color)));
                commands.add(IntoTheDeepCommand.driveDirectToPoseFastCommand(IntoTheDeepPose.NET_PUSH_START_1.getPose(color)));
                commands.add(IntoTheDeepCommand.driveDirectToPoseFastCommand(IntoTheDeepPose.NET_PUSH_TARGET_1.getPose(color)));
                commands.add(IntoTheDeepCommand.driveDirectToPoseFastCommand(IntoTheDeepPose.NET_PUSH_START_1.getPose(color)));
                commands.add(IntoTheDeepCommand.driveDirectToPoseFastCommand(IntoTheDeepPose.NET_PUSH_START_2.getPose(color)));
                commands.add(IntoTheDeepCommand.driveDirectToPoseFastCommand(IntoTheDeepPose.NET_PUSH_TARGET_2.getPose(color)));
                break;
            case START_NET_SCORE_HIGH_COLLECT_SCORE_PARK:
                commands.addAll(IntoTheDeepCommand.dunkSequence(color));
                commands.addAll(IntoTheDeepCommand.collectAndDunkBlockSequence(color, IntoTheDeepFieldPosition.AUTON_BLOCK_NEUTRAL_1));
                commands.addAll(IntoTheDeepCommand.collectAndDunkBlockSequence(color, IntoTheDeepFieldPosition.AUTON_BLOCK_NEUTRAL_2));
                commands.addAll(IntoTheDeepCommand.collectAndDunkBlockSequence(color, IntoTheDeepFieldPosition.AUTON_BLOCK_NEUTRAL_3));
                break;
        }

        Pose2d parkTargetFirst = IntoTheDeepPose.PARK_TARGET_FIRST.getPose(color);
        Pose2d parkTargetSecond = IntoTheDeepPose.PARK_TARGET_SECOND.getPose(color);
        Pose2d parkPose = parkTargetSecond;
        switch (this) {
            case START_OBS_PUSH_NET_PARK:
                parkPose = parkTargetFirst;
                break;
            default:
            case START_NET_PUSH_NET_PARK:
            case START_NET_SCORE_HIGH_PUSH_PARK:
            case START_NET_SCORE_HIGH_COLLECT_SCORE_PARK:
                break;
        }

        commands.add(IntoTheDeepCommand.driveDirectToPoseFastCommand(parkPose));

        return commands;
    }

    public Pose2d getStartingPose(AllianceColor allianceColor) {
        switch (this) {
            case START_OBS_PUSH_NET_PARK:
                return IntoTheDeepPose.START_OBSERVATION_ZONE.getPose(allianceColor);
            case START_NET_PUSH_NET_PARK:
            case START_NET_SCORE_HIGH_PUSH_PARK:
            case START_NET_SCORE_HIGH_COLLECT_SCORE_PARK:
            default:
                return IntoTheDeepPose.START_NET_ZONE.getPose(allianceColor);
        }
    }
}
