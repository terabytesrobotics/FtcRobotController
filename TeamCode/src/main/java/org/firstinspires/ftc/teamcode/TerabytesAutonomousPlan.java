package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

import java.util.ArrayList;
import java.util.List;

public enum TerabytesAutonomousPlan {
    START_NET_PUSH_NET_PARK,
    START_OBS_PUSH_NET_PARK; // TODO: This is a bad plan

    public List<TerabytesCommand> getCommandSequence(AllianceColor color) {
        ArrayList<TerabytesCommand> commands = new ArrayList<>();
        switch (this) {
            case START_OBS_PUSH_NET_PARK:
            case START_NET_PUSH_NET_PARK:
                commands.add(TerabytesCommand.driveDirectToPoseCommand(IntoTheDeepPose.NET_PUSH_STAGING.getPose(color)));
                commands.add(TerabytesCommand.driveDirectToPoseFastCommand(IntoTheDeepPose.NET_PUSH_START_1.getPose(color)));
                commands.add(TerabytesCommand.driveDirectToPoseFastCommand(IntoTheDeepPose.NET_PUSH_TARGET_1.getPose(color)));
                commands.add(TerabytesCommand.driveDirectToPoseFastCommand(IntoTheDeepPose.NET_PUSH_START_1.getPose(color)));
                commands.add(TerabytesCommand.driveDirectToPoseFastCommand(IntoTheDeepPose.NET_PUSH_START_2.getPose(color)));
                commands.add(TerabytesCommand.driveDirectToPoseFastCommand(IntoTheDeepPose.NET_PUSH_TARGET_2.getPose(color)));
                commands.add(TerabytesCommand.driveDirectToPoseFastCommand(IntoTheDeepPose.PARK_TARGET_FIRST.getPose(color)));
                break;
        }

        return commands;
    }

    public Pose2d getStartingPose(AllianceColor allianceColor) {
        switch (this) {
            case START_OBS_PUSH_NET_PARK:
                return IntoTheDeepPose.START_OBSERVATION_ZONE.getPose(allianceColor);
            case START_NET_PUSH_NET_PARK:
            default:
                return IntoTheDeepPose.START_NET_ZONE.getPose(allianceColor);
        }
    }
}
