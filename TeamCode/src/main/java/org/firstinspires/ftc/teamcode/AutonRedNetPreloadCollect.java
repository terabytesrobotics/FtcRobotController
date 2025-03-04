package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

@Config
@Autonomous(name="RED:NET:PRELOAD-SCORE-PARK")
public class AutonRedNetPreloadCollect extends TerabytesOpMode  {

    public AutonRedNetPreloadCollect() {
        super(AllianceColor.RED, IntoTheDeepOpModeState.COMMAND_SEQUENCE, TerabytesAutonomousPlan.START_CLIP_CENTER_SCORE_PARK);
    }
}
