package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

@Config
@Autonomous(name="RED:OBS:WAIT-PRELOAD-WAIT")
public class AutonRedObservationWait extends TerabytesOpMode {

    public AutonRedObservationWait() {
        super(AllianceColor.RED, IntoTheDeepOpModeState.COMMAND_SEQUENCE, TerabytesAutonomousPlan.START_OBS_WAIT_SCORE_WAIT);
    }
}
