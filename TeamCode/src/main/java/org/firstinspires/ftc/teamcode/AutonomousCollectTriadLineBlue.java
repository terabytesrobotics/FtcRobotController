package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

@Autonomous(name = "AUTO Collect Triad Line Blue (Goal Start)", group = "Autonomous")
public class AutonomousCollectTriadLineBlue extends DecodeOpMode {
    public AutonomousCollectTriadLineBlue() {
        super(AllianceColor.BLUE, OpModeState.COMMAND_SEQUENCE, AutonomousPlan.COLLECT_TRIAD_LINE_TEST);
    }
}
