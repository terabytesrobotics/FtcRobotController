package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

@Autonomous(name = "AUTO Shoot-Collect-Shoot Red (Audience)", group = "Autonomous")
public class AutonomousShootCollectShootRedAudienceStart extends DecodeOpMode {

    public AutonomousShootCollectShootRedAudienceStart() {
        super(AllianceColor.RED, OpModeState.COMMAND_SEQUENCE, AutonomousPlan.SHOOT_COLLECT_SHOOT_FROM_AUDIENCE);
    }
}
