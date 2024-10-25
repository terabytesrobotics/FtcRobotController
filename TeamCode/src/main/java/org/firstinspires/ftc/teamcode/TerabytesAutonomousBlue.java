package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

@Config
@Autonomous(name="Terabytes Autonomous: BLUE")
public class TerabytesAutonomousBlue extends TerabytesOpMode {

    public TerabytesAutonomousBlue() {
        super(AllianceColor.BLUE, IntoTheDeepOpModeState.COMMAND_SEQUENCE, TerabytesAutonomousPlan.PARK_IN_OBSERVATION_ZONE);
    }
}
