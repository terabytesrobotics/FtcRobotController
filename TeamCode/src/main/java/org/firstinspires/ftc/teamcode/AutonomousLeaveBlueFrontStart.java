package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

@Autonomous(name = "AUTO Leave Blue (Front Start)", group = "Autonomous")
public class AutonomousLeaveBlueFrontStart extends DecodeOpMode {

    public AutonomousLeaveBlueFrontStart() {
        super(AllianceColor.BLUE, OpModeState.COMMAND_SEQUENCE, AutonomousPlan.LEAVE_FROM_FRONT);
    }
}
