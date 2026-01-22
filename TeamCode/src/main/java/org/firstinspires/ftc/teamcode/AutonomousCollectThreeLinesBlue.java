package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

@Autonomous(name = "AUTO Collect 3 Lines + Shoot Blue", group = "Autonomous")
public class AutonomousCollectThreeLinesBlue extends DecodeOpMode {

    public AutonomousCollectThreeLinesBlue() {
        super(AllianceColor.BLUE, OpModeState.COMMAND_SEQUENCE, AutonomousPlan.COLLECT_THREE_LINES_BLUE);
    }
}
