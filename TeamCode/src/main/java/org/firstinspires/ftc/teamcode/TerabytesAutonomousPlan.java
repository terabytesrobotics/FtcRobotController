package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

import java.util.ArrayList;
import java.util.List;

public enum TerabytesAutonomousPlan {
    PARK_IN_OBSERVATION_ZONE;

    public List<TerabytesCommand> getCommandSequence(AllianceColor color) {
        ArrayList<TerabytesCommand> commands = new ArrayList<>();
        switch (this) {
            case PARK_IN_OBSERVATION_ZONE:
                commands.add(TerabytesCommand.driveDirectToPoseCommand(IntoTheDeepPose.AUTON_PRE_NEUTRAL_PUSH_STAGING.getPose(color)));
                commands.add(TerabytesCommand.driveDirectToPoseFastCommand(IntoTheDeepPose.AUTON_PRE_NEUTRAL_PUSH_1.getPose(color)));
                commands.add(TerabytesCommand.driveDirectToPoseFastCommand(IntoTheDeepPose.AUTON_NEUTRAL_PUSH_TARGET_1.getPose(color)));
                commands.add(TerabytesCommand.driveDirectToPoseFastCommand(IntoTheDeepPose.AUTON_PRE_NEUTRAL_PUSH_1.getPose(color)));
                commands.add(TerabytesCommand.driveDirectToPoseFastCommand(IntoTheDeepPose.AUTON_PRE_NEUTRAL_PUSH_2.getPose(color)));
                commands.add(TerabytesCommand.driveDirectToPoseFastCommand(IntoTheDeepPose.AUTON_NEUTRAL_PUSH_TARGET_2.getPose(color)));
                commands.add(TerabytesCommand.driveDirectToPoseFastCommand(IntoTheDeepPose.AUTON_PARK_TARGET.getPose(color)));
                break;
        }

        return commands;
    }
}
