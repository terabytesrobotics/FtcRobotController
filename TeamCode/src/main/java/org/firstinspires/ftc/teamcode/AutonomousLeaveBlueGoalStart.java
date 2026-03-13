package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

@Autonomous(name = "AUTO Leave Blue (Goal Start)", group = "Autonomous")
@Disabled
public class AutonomousLeaveBlueGoalStart extends DecodeOpMode {

    public AutonomousLeaveBlueGoalStart() {
        super(AllianceColor.BLUE, OpModeState.COMMAND_SEQUENCE, AutonomousPlan.LEAVE_FROM_GOAL);
    }
}
