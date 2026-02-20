package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

@Autonomous(name = "AUTO Shoot 3 + Leave Red (Audience)", group = "Autonomous")
public class AutonomousShootThreeRedAudienceStart extends DecodeOpMode {

    public AutonomousShootThreeRedAudienceStart() {
        super(AllianceColor.RED, OpModeState.COMMAND_SEQUENCE, AutonomousPlan.SHOOT_THREE_FROM_AUDIENCE);
    }
}
