package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.openftc.apriltag.AprilTagDetection;

public abstract class TerabytesOpMode extends LinearOpMode {

    protected final boolean debugMode;
    private final AllianceColor allianceColor;
    private final TerabytesOpModeState startupState;
    private Pose2d startPose = null;
    private TerabytesAutonomousPlan autonomousPlan = null;

    public TerabytesOpMode(AllianceColor allianceColor, TerabytesOpModeState startupState) {
        super();
        this.allianceColor = allianceColor;
        this.startupState = startupState;
        this.debugMode = false;
    }

    public TerabytesOpMode(AllianceColor allianceColor, TerabytesOpModeState startupState, TerabytesAutonomousPlan autonomousPlan) {
        super();
        this.allianceColor = allianceColor;
        this.startupState = startupState;
        this.autonomousPlan = autonomousPlan;
        this.startPose = allianceColor.getStartingPose(autonomousPlan);
        this.debugMode = false;
    }

    public TerabytesOpMode(AllianceColor allianceColor, TerabytesOpModeState startupState, TerabytesAutonomousPlan autonomousPlan, boolean debugMode) {
        super();
        this.allianceColor = allianceColor;
        this.startupState = startupState;
        this.autonomousPlan = autonomousPlan;
        this.startPose = allianceColor.getStartingPose(autonomousPlan);
        this.debugMode = debugMode;
    }

    @Override
    public void runOpMode() {
        // Hooks up telemetry data to the dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        TerabytesIntoTheDeep terabytes = new TerabytesIntoTheDeep(
                allianceColor,
                gamepad1,
                gamepad2,
                hardwareMap,
                telemetry,
                debugMode);
        boolean isSlowInit = startPose != null;
        if (isSlowInit) {
            terabytes.autonomousInit(startPose, autonomousPlan);
        } else {
            terabytes.teleopInit();
        }
        waitForStart();

        if (!isStopRequested()) {
            terabytes.startup(startupState);
        }

        while (!isStopRequested() && terabytes.evaluate()) {}
        terabytes.shutDown();
    }
}
