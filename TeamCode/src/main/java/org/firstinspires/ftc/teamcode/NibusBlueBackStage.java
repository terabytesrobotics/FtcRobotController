package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.AlliancePose;

@Config
@TeleOp(name="Blue BackStage Start")
public class NibusBlueBackStage extends TeleOpNibus2000 {

    public NibusBlueBackStage() {
        super(AllianceColor.BLUE, AlliancePose.BACKSTAGE_START);
    }
}
