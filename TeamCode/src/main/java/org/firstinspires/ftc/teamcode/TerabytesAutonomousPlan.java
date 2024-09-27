package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

import java.util.ArrayList;
import java.util.List;

public enum TerabytesAutonomousPlan {
    PARK_IN_OBSERVATION_ZONE;

    public List<TerabytesCommand> getCommandSequence(AllianceColor color) {
        ArrayList<TerabytesCommand> commands = new ArrayList<>();
        switch (this) {
            case PARK_IN_OBSERVATION_ZONE:
                switch (color) {
                    case RED:
                        commands.add(TerabytesCommand.driveDirectToPoseFastCommand(new Pose2d(56, -50, 3*Math.PI/2)));
                        break;
                    case BLUE:
                        commands.add(TerabytesCommand.driveDirectToPoseFastCommand(new Pose2d(-56, 50,Math.PI/2)));
                        break;
                }
                break;
        }

        return commands;
    }
}
