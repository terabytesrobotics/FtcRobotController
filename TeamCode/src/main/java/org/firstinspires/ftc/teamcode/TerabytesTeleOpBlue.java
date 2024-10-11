package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

@Config
@TeleOp(name="Terabytes TeleOp: BLUE")
public class TerabytesTeleOpBlue extends TerabytesOpMode {

    public TerabytesTeleOpBlue() {
        super(AllianceColor.BLUE, TerabytesOpModeState.HEADLESS_DRIVE);
    }
}