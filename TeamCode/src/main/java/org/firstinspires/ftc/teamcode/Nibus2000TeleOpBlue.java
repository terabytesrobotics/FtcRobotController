package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

@Config
@TeleOp(name="NIBUS 2000: Blue edition")
public class Nibus2000TeleOpBlue extends Nibus2000OpMode {

    public Nibus2000TeleOpBlue() {
        super(AllianceColor.BLUE);
    }
}
