package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

@Config
@Autonomous(name="Red Backstage")
public class RedBackstage extends Nibus2000OpMode {

    public RedBackstage() {
        super(AllianceColor.RED, NibusState.DETECT_ALLIANCE_MARKER, NibusAutonomousPlan.START_BACKSTAGE);
    }
}
