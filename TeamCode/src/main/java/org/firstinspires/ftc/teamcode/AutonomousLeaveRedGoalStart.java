package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

@Autonomous(name = "AUTO Leave Red (Goal Start)", group = "Autonomous")
@Disabled
public class AutonomousLeaveRedGoalStart extends DecodeOpMode {

    public AutonomousLeaveRedGoalStart() {
        super(AllianceColor.RED, OpModeState.COMMAND_SEQUENCE, AutonomousPlan.LEAVE_FROM_GOAL);
    }
}
