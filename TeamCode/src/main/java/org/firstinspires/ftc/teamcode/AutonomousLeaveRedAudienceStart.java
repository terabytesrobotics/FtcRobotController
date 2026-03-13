package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

@Autonomous(name = "AUTO Leave Red (Audience Start)", group = "Autonomous")
@Disabled
public class AutonomousLeaveRedAudienceStart extends DecodeOpMode {

    public AutonomousLeaveRedAudienceStart() {
        super(AllianceColor.RED, OpModeState.COMMAND_SEQUENCE, AutonomousPlan.LEAVE_FROM_AUDIENCE);
    }
}
