package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.AlliancePose;

@Config
@Autonomous(name="Red FrontStage Start")
public class NibusRedFrontStage extends Nibus2000OpMode {

    public NibusRedFrontStage() {
        super(AllianceColor.RED, AlliancePose.FRONTSTAGE_START);
    }
}
