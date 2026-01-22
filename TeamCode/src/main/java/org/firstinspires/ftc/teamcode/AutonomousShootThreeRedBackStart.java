package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

@Autonomous(name = "AUTO Shoot 3 + Park Red (Back)", group = "Autonomous")
public class AutonomousShootThreeRedBackStart extends DecodeOpMode {

    public AutonomousShootThreeRedBackStart() {
        super(AllianceColor.RED, OpModeState.COMMAND_SEQUENCE, AutonomousPlan.SHOOT_THREE_FROM_BACK);
    }
}
