package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

@Autonomous(name = "AUTO Leave Red (Corner Start)", group = "Autonomous")
public class AutonomousLeaveRedCornerStart extends DecodeOpMode {

    public AutonomousLeaveRedCornerStart() {
        super(AllianceColor.RED, OpModeState.COMMAND_SEQUENCE, AutonomousPlan.LEAVE_FROM_CORNER);
    }
}
