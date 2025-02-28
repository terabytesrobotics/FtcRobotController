package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

@Config
@Autonomous(name="RED:CLIP:CENTER:PRELOAD-SCORE-PARK")
public class AutonRedClipCenterPreloadCollect extends TerabytesOpMode  {

    public AutonRedClipCenterPreloadCollect() {
        super(AllianceColor.RED, IntoTheDeepOpModeState.COMMAND_SEQUENCE, TerabytesAutonomousPlan.STARTCLIP_CENTER_SCORE_PARK);
    }
}
