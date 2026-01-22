package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

@Autonomous(name = "AUTO Shoot 3 + Leave Blue (Long)", group = "Autonomous")
public class AutonomousShootThreeBlueCornerStart extends DecodeOpMode {

    public AutonomousShootThreeBlueCornerStart() {
        super(AllianceColor.BLUE, OpModeState.COMMAND_SEQUENCE, AutonomousPlan.SHOOT_THREE_FROM_CORNER);
    }
}
