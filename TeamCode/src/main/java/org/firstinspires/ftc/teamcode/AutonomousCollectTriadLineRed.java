package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

@Autonomous(name = "AUTO Collect Triad Line Red (Goal Start)", group = "Autonomous")
public class AutonomousCollectTriadLineRed extends DecodeOpMode {
    public AutonomousCollectTriadLineRed() {
        super(AllianceColor.RED, OpModeState.COMMAND_SEQUENCE, AutonomousPlan.COLLECT_TRIAD_LINE_TEST);
    }
}
