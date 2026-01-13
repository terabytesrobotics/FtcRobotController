package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

@Autonomous(name = "AUTO Leave Blue (Corner Start)", group = "Autonomous")
public class AutonomousLeaveBlueCornerStart extends DecodeOpMode {

    public AutonomousLeaveBlueCornerStart() {
        super(AllianceColor.BLUE, OpModeState.COMMAND_SEQUENCE, AutonomousPlan.LEAVE_FROM_CORNER);
    }
}
