package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

@Config
@Autonomous(name="BLUE:NET")
public class TerabytesAutonomousBlueNet extends TerabytesOpMode {

    public TerabytesAutonomousBlueNet() {
        super(AllianceColor.BLUE, IntoTheDeepOpModeState.COMMAND_SEQUENCE, TerabytesAutonomousPlan.START_NET_PUSH_NET_PARK);
    }
}