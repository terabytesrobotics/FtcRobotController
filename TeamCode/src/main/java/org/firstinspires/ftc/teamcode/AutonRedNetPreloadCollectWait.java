package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

@Config
@Autonomous(name="RED:NET:PRELOAD-SCORE-WAIT")
public class AutonRedNetPreloadCollectWait extends TerabytesOpMode {

    public AutonRedNetPreloadCollectWait() {
        super(AllianceColor.RED, IntoTheDeepOpModeState.COMMAND_SEQUENCE, TerabytesAutonomousPlan.START_NET_SCORE_COLLECT_SCORE_WAIT);
    }
}
