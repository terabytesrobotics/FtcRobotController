package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.AlliancePose;

@Config
@Autonomous(name="Blue Interstitial Test")
@Disabled
public class NibusBlueInterstitialTest extends Nibus2000OpMode {

    public NibusBlueInterstitialTest() {
        super(AllianceColor.BLUE, AlliancePose.BACKSTAGE_START, NibusState.MANUAL_DRIVE, null);
    }
}
