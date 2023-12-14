package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.AlliancePose;

@Config
@TeleOp(name="Red FrontStage Start")
public class NibusRedFrontStage extends TeleOpNibus2000 {

    public NibusRedFrontStage() {
        super(AllianceColor.RED, AlliancePose.FRONTSTAGE_START);
    }
}
