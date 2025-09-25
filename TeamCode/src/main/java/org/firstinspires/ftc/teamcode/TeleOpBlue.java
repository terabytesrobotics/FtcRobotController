package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

@Config
@TeleOp(name="BLUE")
public class TeleOpBlue extends DecodeOpMode {

    public TeleOpBlue() {
        super(AllianceColor.BLUE, OpModeState.MANUAL_CONTROL);
    }
}
