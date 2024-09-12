package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import java.util.ArrayList;
import java.util.List;

public enum TerabytesAutonomousPlan {
    ONE,
    TWO;

    public List<TerabytesCommand> getCommandSequence() {
        ArrayList<TerabytesCommand> commands = new ArrayList<>();
        switch (this) {
            case ONE:
                commands.add(TerabytesCommand.driveDirectToPoseFastCommand(new Pose2d(0, 24)));
                commands.add(TerabytesCommand.driveDirectToPoseFastCommand(new Pose2d(0, -24)));
                commands.add(TerabytesCommand.driveDirectToPoseCommand(new Pose2d(0, 0)));
                break;
            case TWO:
                commands.add(TerabytesCommand.driveDirectToPoseFastCommand(new Pose2d(24, 0)));
                commands.add(TerabytesCommand.driveDirectToPoseFastCommand(new Pose2d(-24, 0)));
                commands.add(TerabytesCommand.driveDirectToPoseCommand(new Pose2d(0, 0)));
                break;
        }

        return commands;
    }
}
