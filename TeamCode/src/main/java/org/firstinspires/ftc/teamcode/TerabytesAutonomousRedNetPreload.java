package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

@Config
@Autonomous(name="RED:NET:PRELOAD")
public class TerabytesAutonomousRedNetPreload extends TerabytesOpMode {

    public TerabytesAutonomousRedNetPreload() {
        super(AllianceColor.RED, IntoTheDeepOpModeState.COMMAND_SEQUENCE, TerabytesAutonomousPlan.START_NET_SCORE_HIGH_PUSH_PARK);
    }
}
