package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

@Config
@Autonomous(name="Blue Backstage")
@Disabled
public class BlueBackstage extends Nibus2000OpMode {

    public BlueBackstage() {
        super(AllianceColor.BLUE, NibusState.DETECT_ALLIANCE_MARKER, NibusAutonomousPlan.START_BACKSTAGE);
    }
}
