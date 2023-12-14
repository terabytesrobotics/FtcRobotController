package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.AlliancePose;

@Config
@TeleOp(name="Red BackStage Start")
public class NibusRedBackStage extends TeleOpNibus2000 {

    public NibusRedBackStage() {
        super(AllianceColor.RED, AlliancePose.BACKSTAGE_START);
    }
}
