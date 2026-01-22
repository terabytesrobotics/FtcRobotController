package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

@Autonomous(name = "AUTO Shoot 3 + Park Blue (Back)", group = "Autonomous")
public class AutonomousShootThreeBlueBackStart extends DecodeOpMode {

    public AutonomousShootThreeBlueBackStart() {
        super(AllianceColor.BLUE, OpModeState.COMMAND_SEQUENCE, AutonomousPlan.SHOOT_THREE_FROM_BACK);
    }
}
