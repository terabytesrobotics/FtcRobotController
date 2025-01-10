package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

@Config
@Autonomous(name="BLUE:OBS:WAIT-PRELOAD-WAIT")
public class AutonBlueObservationWait extends TerabytesOpMode {

    public AutonBlueObservationWait() {
        super(AllianceColor.BLUE, IntoTheDeepOpModeState.COMMAND_SEQUENCE, TerabytesAutonomousPlan.START_OBS_WAIT_SCORE_WAIT);
    }
}
