package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

@Autonomous(name = "AUTO Shoot 3 + Park Blue (Goal)", group = "Autonomous")
public class AutonomousShootThreeBlueGoalStart extends DecodeOpMode {

    public AutonomousShootThreeBlueGoalStart() {
        super(AllianceColor.BLUE, OpModeState.COMMAND_SEQUENCE, AutonomousPlan.SHOOT_THREE_FROM_GOAL);
    }
}
