package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

@Autonomous(name = "AUTO Shoot 3 + Park Red (Goal)", group = "Autonomous")
public class AutonomousShootThreeRedGoalStart extends DecodeOpMode {

    public AutonomousShootThreeRedGoalStart() {
        super(AllianceColor.RED, OpModeState.COMMAND_SEQUENCE, AutonomousPlan.SHOOT_THREE_FROM_GOAL);
    }
}
