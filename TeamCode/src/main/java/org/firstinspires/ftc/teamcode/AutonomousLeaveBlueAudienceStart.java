package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

@Autonomous(name = "AUTO Leave Blue (Audience Start)", group = "Autonomous")
@Disabled
public class AutonomousLeaveBlueAudienceStart extends DecodeOpMode {

    public AutonomousLeaveBlueAudienceStart() {
        super(AllianceColor.BLUE, OpModeState.COMMAND_SEQUENCE, AutonomousPlan.LEAVE_FROM_AUDIENCE);
    }
}
