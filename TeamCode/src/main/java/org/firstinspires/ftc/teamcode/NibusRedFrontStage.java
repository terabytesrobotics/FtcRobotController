package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

@Config
@Autonomous(name="Red FrontStage Start")
public class NibusRedFrontStage extends Nibus2000OpMode {

    public NibusRedFrontStage() {
        super(AllianceColor.RED, NibusState.DETECT_ALLIANCE_MARKER, NibusAutonomousPlan.START_FRONTSTAGE);
    }
}
