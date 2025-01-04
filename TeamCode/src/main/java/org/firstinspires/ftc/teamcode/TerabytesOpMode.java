package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.util.AllianceColor;

import java.io.File;

public abstract class TerabytesOpMode extends LinearOpMode {

    private static final long SAVE_INTERVAL_MS = 125;
    private long lastSaveTime = 0;
    private static final String PERSISTED_DATA_FILE_NAME = "last_pose.txt";

    private boolean debugMode = false;
    private final AllianceColor allianceColor;
    private final IntoTheDeepOpModeState startupState;
    private TerabytesAutonomousPlan autonomousPlan = null;

    public TerabytesOpMode(AllianceColor allianceColor, IntoTheDeepOpModeState startupState) {
        super();
        this.allianceColor = allianceColor;
        this.startupState = startupState;
    }

    public TerabytesOpMode(AllianceColor allianceColor, IntoTheDeepOpModeState startupState, TerabytesAutonomousPlan autonomousPlan) {
        super();
        this.allianceColor = allianceColor;
        this.startupState = startupState;
        this.autonomousPlan = autonomousPlan;
    }

    public TerabytesOpMode(AllianceColor allianceColor, IntoTheDeepOpModeState startupState, boolean debugMode) {
        super();
        this.allianceColor = allianceColor;
        this.startupState = startupState;
        this.debugMode = debugMode;
    }

    private void savePersistedData(Pose2d pose, int armLTickPosition, int armRTickPosition, int extenderTickPosition) {
        long timestamp = System.currentTimeMillis();
        try {
            File file = AppUtil.getInstance().getSettingsFile(PERSISTED_DATA_FILE_NAME);
            String data = timestamp + "\n" + pose.getX() + "\n" + pose.getY() + "\n" + pose.getHeading() + "\n" + armLTickPosition + "\n" + armRTickPosition + "\n" + extenderTickPosition;
            ReadWriteFile.writeFile(file, data);
        } catch (Exception e) {
            telemetry.log().add("Failed to save: " + e.getMessage());
        }
    }

    private static class PersistedData {
        public Pose2d pose;
        public int armLTicks;
        public int armRTicks;
        public int extenderTicks;
        public long timestamp;
    }

    private PersistedData readAndDeleteLastPersistedData() {
        PersistedData persistedData = null;
        try {
            File poseFile = AppUtil.getInstance().getSettingsFile(PERSISTED_DATA_FILE_NAME);
            if (poseFile.exists()) {
                String[] lines = ReadWriteFile.readFile(poseFile).split("\n");
                if (lines.length >= 4) {
                    long timestamp = Long.parseLong(lines[0]);
                    double x = Double.parseDouble(lines[1]);
                    double y = Double.parseDouble(lines[2]);
                    double heading = Double.parseDouble(lines[3]);
                    int armLTicks = Integer.parseInt(lines[4]);
                    int armRTicks = Integer.parseInt(lines[5]);
                    int extenderTicks = Integer.parseInt(lines[6]);
                    Pose2d pose = new Pose2d(x, y, heading);
                    persistedData = new PersistedData();
                    persistedData.pose = pose;
                    persistedData.armLTicks = armLTicks;
                    persistedData.armRTicks = armRTicks;
                    persistedData.extenderTicks = extenderTicks;
                    persistedData.timestamp = timestamp;
                }
                boolean cleanedUp = poseFile.delete();
            }
        } catch (Exception e) {
        }
        return persistedData;
    }

    @Override
    public void runOpMode() {
        // Hooks up telemetry data to the dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TerabytesIntoTheDeep terabytes = new TerabytesIntoTheDeep(
                allianceColor,
                gamepad1,
                gamepad2,
                hardwareMap,
                debugMode);

        if (autonomousPlan != null) {
            terabytes.autonomousInit(autonomousPlan);
        } else {
            PersistedData persistedData = readAndDeleteLastPersistedData();
            if (debugMode || persistedData == null) {
                terabytes.teleopInit(new Pose2d());
            } else {
                terabytes.teleopInit(
                        persistedData.pose,
                        persistedData.armLTicks,
                        persistedData.armRTicks,
                        persistedData.extenderTicks);
            }
        }

        terabytes.initializeMechanicalBlocking();

        waitForStart();

        if (!isStopRequested()) {
            terabytes.startup(startupState);
        }

        lastSaveTime = System.currentTimeMillis();

        while (!isStopRequested() && terabytes.evaluate()) {
            long currentTime = System.currentTimeMillis();
            if (currentTime - lastSaveTime >= SAVE_INTERVAL_MS) {
                savePersistedData(
                        terabytes.getLatestPoseEstimate(),
                        terabytes.getArmLTickPosition(),
                        terabytes.getArmRTickPosition(),
                        terabytes.getExtenderTickPosition());
                lastSaveTime = currentTime;
            }
            dashboard.sendTelemetryPacket(terabytes.getTelemetryPacket());
        }

        // One last time before shutdown.
        savePersistedData(
                terabytes.getLatestPoseEstimate(),
                terabytes.getArmLTickPosition(),
                terabytes.getArmRTickPosition(),
                terabytes.getExtenderTickPosition());

        terabytes.shutDown();
    }
}
