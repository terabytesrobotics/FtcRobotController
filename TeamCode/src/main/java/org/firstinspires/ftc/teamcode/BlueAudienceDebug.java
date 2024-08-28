package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

@Config
@Autonomous(name="Debug: Blue Audience")
public class BlueAudienceDebug extends Nibus2000OpMode {

    public BlueAudienceDebug() {
        super(AllianceColor.BLUE, NibusState.DETECT_ALLIANCE_MARKER, NibusAutonomousPlan.START_FRONTSTAGE, true);
    }
}
