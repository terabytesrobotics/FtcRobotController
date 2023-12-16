package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.AlliancePose;

@Config
@Autonomous(name="Blue BackStage Start")
public class NibusBlueBackStage extends Nibus2000OpMode {

    public NibusBlueBackStage() {
        super(AllianceColor.BLUE, AlliancePose.BACKSTAGE_START, NibusState.DETECT_ALLIANCE_MARKER, NibusAutonomousPlan.PARK_LEFT);
    }
}
