package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.AlliancePose;

@Config
@TeleOp(name="Blue FrontStage Start")
public class NibusBlueFrontStage extends TeleOpNibus2000 {

    public NibusBlueFrontStage() {
        super(AllianceColor.BLUE, AlliancePose.FRONTSTAGE_START);
    }
}
