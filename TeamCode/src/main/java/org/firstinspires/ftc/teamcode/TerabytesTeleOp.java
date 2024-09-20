package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

@Config
@TeleOp(name="AAA Terabytes TeleOp")
public class TerabytesTeleOp extends TerabytesOpMode {

    public TerabytesTeleOp() {
        super(AllianceColor.RED, TerabytesOpModeState.HEADLESS_DRIVE);
    }
}
