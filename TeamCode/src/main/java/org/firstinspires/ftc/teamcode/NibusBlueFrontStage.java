package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.AlliancePose;

@Config
@Autonomous(name="Blue FrontStage Start")
public class NibusBlueFrontStage extends Nibus2000OpMode {

    public NibusBlueFrontStage() {
        super(AllianceColor.BLUE, AlliancePose.FRONTSTAGE_START);
    }
}
