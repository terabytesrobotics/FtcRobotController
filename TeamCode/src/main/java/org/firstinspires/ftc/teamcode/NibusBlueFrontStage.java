package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.UpstageBackstageStart;

@Config
@Autonomous(name="Blue FrontStage Start")
public class NibusBlueFrontStage extends Nibus2000OpMode {

    public NibusBlueFrontStage() {
        super(AllianceColor.BLUE, UpstageBackstageStart.FRONTSTAGE_START, NibusState.DETECT_ALLIANCE_MARKER, NibusAutonomousPlan.START_UPSTAGE);
    }
}
