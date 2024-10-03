package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.util.AllianceColor;

import java.io.File;

public abstract class TerabytesOpMode extends LinearOpMode {

    private static final long SAVE_INTERVAL_MS = 250; // Save every quarter second
    private long lastSaveTime = 0;
    private static final String POSE_FILE_NAME = "last_pose.txt";
    private static final long FRESHNESS_LIMIT_MS = 30000; // 30 seconds

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

    private void saveLastPose(Pose2d pose) {
        long timestamp = System.currentTimeMillis();
        try {
            File poseFile = AppUtil.getInstance().getSettingsFile(POSE_FILE_NAME);
            String data = timestamp + "\n" + pose.getX() + "\n" + pose.getY() + "\n" + pose.getHeading();
            ReadWriteFile.writeFile(poseFile, data);
        } catch (Exception e) {
            telemetry.log().add("Failed to save last pose: " + e.getMessage());
        }
    }

    private class SavedPose {
        public Pose2d pose;
        public long timestamp;
    }

    private SavedPose readLastPose() {
        SavedPose savedPose = null;
        try {
            File poseFile = AppUtil.getInstance().getSettingsFile(POSE_FILE_NAME);
            if (poseFile.exists()) {
                String[] lines = ReadWriteFile.readFile(poseFile).split("\n");
                if (lines.length >= 4) {
                    long timestamp = Long.parseLong(lines[0]);
                    double x = Double.parseDouble(lines[1]);
                    double y = Double.parseDouble(lines[2]);
                    double heading = Double.parseDouble(lines[3]);
                    Pose2d pose = new Pose2d(x, y, heading);
                    savedPose = new SavedPose();
                    savedPose.pose = pose;
                    savedPose.timestamp = timestamp;
                }
            }
        } catch (Exception e) {
            telemetry.log().add("Failed to read last pose: " + e.getMessage());
        }
        return savedPose;
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

        boolean isAutonomous = startPose != null;

        // Attempt to read the last saved pose if no startPose is provided
        if (startPose == null) {
            SavedPose savedPose = readLastPose();
            if (savedPose != null) {
                long currentTime = System.currentTimeMillis();
                if (currentTime - savedPose.timestamp <= FRESHNESS_LIMIT_MS) {
                    startPose = savedPose.pose;
                    telemetry.addData("Using saved start pose", startPose);
                }
            }
        }

        if (isAutonomous) {
            terabytes.autonomousInit(startPose, autonomousPlan);
        } else {
            terabytes.teleopInit(startPose);
        }
        waitForStart();

        if (!isStopRequested()) {
            terabytes.startup(startupState);
        }

        lastSaveTime = System.currentTimeMillis();

        while (!isStopRequested() && terabytes.evaluate()) {
            long currentTime = System.currentTimeMillis();
            if (currentTime - lastSaveTime >= SAVE_INTERVAL_MS) {
                Pose2d currentPose = terabytes.getLatestPoseEstimate();
                if (currentPose != null) {
                    saveLastPose(currentPose);
                    lastSaveTime = currentTime;
                }
            }
        }
        terabytes.shutDown();
    }
}
