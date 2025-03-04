package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

@Config
@Autonomous(name="BLUE:CLIP:OBS:PRELOAD-SCORE-PARK")
public class AutonBlueObservationClip extends TerabytesOpMode  {

    public AutonBlueObservationClip() {
        super(AllianceColor.BLUE, IntoTheDeepOpModeState.COMMAND_SEQUENCE, TerabytesAutonomousPlan.START_OBS_SCORE_CLIPS);
    }
}
