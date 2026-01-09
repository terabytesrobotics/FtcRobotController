package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

    @Autonomous(name = "AUTO Leave Blue", group = "Autonomous")
    public class AutonomousLeaveBlue extends DecodeOpMode {

        public AutonomousLeaveBlue() {
            super(AllianceColor.BLUE, OpModeState.COMMAND_SEQUENCE, AutonomousPlan.SIMPLE_AUTON);
        }
    }
