package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

@Config
@TeleOp(name="DEBUG: *RUN SAFELY*")
public class TeleOpDebug extends DecodeOpMode {

    public TeleOpDebug() {
        super(AllianceColor.RED, OpModeState.MANUAL_CONTROL, true);
    }
}
