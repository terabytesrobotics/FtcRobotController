package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

@Config
@Autonomous(name="RED:CLIP:OBS:PRELOAD-SCORE3")
public class AutonRedObservation3Clip extends TerabytesOpMode  {

    public AutonRedObservation3Clip() {
        super(AllianceColor.RED, IntoTheDeepOpModeState.COMMAND_SEQUENCE, TerabytesAutonomousPlan.START_OBS_SCORE_3CLIPS);
    }
}
