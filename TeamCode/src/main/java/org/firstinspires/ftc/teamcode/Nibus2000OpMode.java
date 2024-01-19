package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.UpstageBackstageStart;
import org.firstinspires.ftc.teamcode.util.BlueGrabberState;
import org.firstinspires.ftc.teamcode.util.CollectorState;
import org.firstinspires.ftc.teamcode.util.GreenGrabberState;

class NibusSaveState {

    public final Pose2d Pose;
    public final int ArmPosition;
    public final int ExtenderPosition;
    public final CollectorState CollectorState;
    public final BlueGrabberState BlueGrabberState;
    public final GreenGrabberState GreenGrabberState;

    public NibusSaveState(Pose2d pose, int armPosition, int extenderPosition, CollectorState collectorState, BlueGrabberState blueGrabberState, GreenGrabberState greenGrabberState) {
        Pose = pose;
        ArmPosition = armPosition;
        ExtenderPosition = extenderPosition;
        CollectorState = collectorState;
        BlueGrabberState = blueGrabberState;
        GreenGrabberState = greenGrabberState;
    }
}

public abstract class Nibus2000OpMode extends LinearOpMode {

    private final AllianceColor allianceColor;
    private final NibusState startupState;
    private UpstageBackstageStart alliancePose = null;
    private NibusAutonomousPlan autonomousPlan = null;


    public Nibus2000OpMode(AllianceColor allianceColor, NibusState startupState) {
        super();
        this.allianceColor = allianceColor;
        this.startupState = startupState;
    }

    public Nibus2000OpMode(AllianceColor allianceColor, NibusState startupState, NibusAutonomousPlan autonomousPlan) {
        super();
        this.allianceColor = allianceColor;
        this.startupState = startupState;
        this.autonomousPlan = autonomousPlan;
    }

    @Override
    public void runOpMode() {
        Nibus2000 nibus = new Nibus2000(
                allianceColor,
                gamepad1,
                gamepad2,
                hardwareMap,
                telemetry);
        boolean isSlowInit = alliancePose != null;
        if (isSlowInit) {
            nibus.autonomousInit(alliancePose, autonomousPlan);
        } else {
            nibus.teleopInit();
        }
        waitForStart();
        nibus.startup(startupState);

        while (!isStopRequested() && nibus.evaluate()) {}
        nibus.shutDown();
    }
}
