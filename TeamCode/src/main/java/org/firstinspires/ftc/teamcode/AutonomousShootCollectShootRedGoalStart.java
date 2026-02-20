package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

@Autonomous(name = "AUTO Shoot-Collect-Shoot Red (Goal)", group = "Autonomous")
public class AutonomousShootCollectShootRedGoalStart extends DecodeOpMode {

    public AutonomousShootCollectShootRedGoalStart() {
        super(AllianceColor.RED, OpModeState.COMMAND_SEQUENCE, AutonomousPlan.SHOOT_COLLECT_SHOOT_FROM_GOAL);
    }
}
