package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.AlliancePose;

@Config
@Autonomous(name="Red Interstitial Test")
public class NibusRedInterstitialTest extends Nibus2000OpMode {

    public NibusRedInterstitialTest() {
        super(AllianceColor.RED, AlliancePose.BACKSTAGE_START, NibusState.MANUAL_DRIVE);
    }
}
