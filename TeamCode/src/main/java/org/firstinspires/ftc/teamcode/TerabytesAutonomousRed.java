package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

@Config
@Autonomous(name="Terabytes Autonomous: RED")
public class TerabytesAutonomousRed extends TerabytesOpMode {

    public TerabytesAutonomousRed() {
        super(AllianceColor.RED, TerabytesOpModeState.COMMAND_SEQUENCE, TerabytesAutonomousPlan.PARK_IN_OBSERVATION_ZONE);
    }
}
