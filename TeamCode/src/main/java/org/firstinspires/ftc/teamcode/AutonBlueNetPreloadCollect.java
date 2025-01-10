package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

@Config
@Autonomous(name="BLUE:NET:PRELOAD-SCORE-PARK")
public class AutonBlueNetPreloadCollect extends TerabytesOpMode  {

    public AutonBlueNetPreloadCollect() {
        super(AllianceColor.BLUE, IntoTheDeepOpModeState.COMMAND_SEQUENCE, TerabytesAutonomousPlan.START_NET_SCORE_COLLECT_SCORE_PARK);
    }
}
