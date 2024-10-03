package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

@Config
@TeleOp(name="Terabytes TeleOp: RED")
public class TerabytesTeleOpRed extends TerabytesOpMode {

    public TerabytesTeleOpRed() {
        super(AllianceColor.RED, TerabytesOpModeState.HEADLESS_DRIVE);
    }
}
