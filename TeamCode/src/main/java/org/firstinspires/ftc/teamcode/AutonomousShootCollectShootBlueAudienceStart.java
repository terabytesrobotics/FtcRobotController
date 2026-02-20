package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

@Autonomous(name = "AUTO Shoot-Collect-Shoot Blue (Audience)", group = "Autonomous")
public class AutonomousShootCollectShootBlueAudienceStart extends DecodeOpMode {

    public AutonomousShootCollectShootBlueAudienceStart() {
        super(AllianceColor.BLUE, OpModeState.COMMAND_SEQUENCE, AutonomousPlan.SHOOT_COLLECT_SHOOT_FROM_AUDIENCE);
    }
}
