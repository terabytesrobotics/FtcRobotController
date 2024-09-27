package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

@Config
@Autonomous(name="AAA Terabytes Autonomous")
public class TerabytesAutonomous extends TerabytesOpMode {

    public TerabytesAutonomous() {
        super(AllianceColor.RED, TerabytesOpModeState.COMMAND_SEQUENCE, TerabytesAutonomousPlan.PARK_IN_OBSERVATION_ZONE);
    }
}
