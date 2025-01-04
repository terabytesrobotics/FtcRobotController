package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

@Config
@TeleOp(name="DEBUG: *RUN SAFELY*")
public class TerabytesTeleOpDebug extends TerabytesOpMode {

    public TerabytesTeleOpDebug() {
        super(AllianceColor.RED, IntoTheDeepOpModeState.MANUAL_CONTROL, true);
    }
}
