package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

@Autonomous(name = "AUTO Leave Red (Front Start)", group = "Autonomous")
public class AutonomousLeaveRedFrontStart extends DecodeOpMode {

    public AutonomousLeaveRedFrontStart() {
        super(AllianceColor.RED, OpModeState.COMMAND_SEQUENCE, AutonomousPlan.LEAVE_FROM_FRONT);
    }
}
