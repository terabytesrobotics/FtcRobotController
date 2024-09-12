package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

@Config
@TeleOp(name="NIBUS 2000: Red edition")
@Disabled
public class Nibus2000TeleOpRed extends Nibus2000OpMode {

    public Nibus2000TeleOpRed() {
        super(AllianceColor.RED, NibusState.MANUAL_DRIVE);
    }
}
