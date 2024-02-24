package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

@Config
@Autonomous(name="Red Audience")
public class RedAudience extends Nibus2000OpMode {

    public RedAudience() {
        super(AllianceColor.RED, NibusState.DETECT_ALLIANCE_MARKER, NibusAutonomousPlan.START_FRONTSTAGE);
    }
}
