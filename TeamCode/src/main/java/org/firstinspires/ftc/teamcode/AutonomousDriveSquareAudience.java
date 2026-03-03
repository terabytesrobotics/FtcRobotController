package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

@Autonomous(name = "AUTO Drive Square (Audience)", group = "Autonomous")
public class AutonomousDriveSquareAudience extends DecodeOpMode {
    public AutonomousDriveSquareAudience() {
        super(AllianceColor.RED, OpModeState.COMMAND_SEQUENCE, AutonomousPlan.DRIVE_SQUARE_TEST);
    }
}
