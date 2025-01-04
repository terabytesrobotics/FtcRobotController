package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

import java.util.ArrayList;
import java.util.List;

public enum TerabytesAutonomousPlan {
    PUSH_NET_PARK_IN_OBSERVATION_ZONE;

    public List<TerabytesCommand> getCommandSequence(AllianceColor color) {
        ArrayList<TerabytesCommand> commands = new ArrayList<>();
        switch (this) {
            case PUSH_NET_PARK_IN_OBSERVATION_ZONE:
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
            case PUSH_NET_PARK_IN_OBSERVATION_ZONE:
            default:
                return IntoTheDeepPose.START_MID.getPose(allianceColor);
        }
    }
}
