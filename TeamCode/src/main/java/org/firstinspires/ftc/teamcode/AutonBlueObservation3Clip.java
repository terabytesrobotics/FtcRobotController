package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

@Config
@Autonomous(name="BLUE:CLIP:OBS:PRELOAD-SCORE3")
public class AutonBlueObservation3Clip extends TerabytesOpMode  {

    public AutonBlueObservation3Clip() {
        super(AllianceColor.BLUE, IntoTheDeepOpModeState.COMMAND_SEQUENCE, TerabytesAutonomousPlan.START_OBS_SCORE_3CLIPS);
    }
}
