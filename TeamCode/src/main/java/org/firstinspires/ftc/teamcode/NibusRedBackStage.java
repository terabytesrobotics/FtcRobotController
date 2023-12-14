package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.AlliancePose;

@Config
@Autonomous(name="Red BackStage Start")
public class NibusRedBackStage extends Nibus2000OpMode {

    public NibusRedBackStage() {
        super(AllianceColor.RED, AlliancePose.BACKSTAGE_START, NibusState.DETECT_ALLIANCE_MARKER);
    }
}
