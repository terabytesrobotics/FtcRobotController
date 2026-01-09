package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

    @Autonomous(name = "AUTO Leave Red", group = "Autonomous")
    public class AutonomousLeaveRed extends DecodeOpMode {

        public AutonomousLeaveRed() {
            super(AllianceColor.RED, OpModeState.COMMAND_SEQUENCE, AutonomousPlan.SIMPLE_AUTON);
        }
    }
