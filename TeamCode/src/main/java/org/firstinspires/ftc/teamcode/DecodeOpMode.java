package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.util.AllianceColor;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;

public abstract class DecodeOpMode extends LinearOpMode {

    private static final long EXPIRY_INTERVAL_MS = 150000; // 2.5 minutes
    private static final long SAVE_INTERVAL_MS = 125;
    private long lastSaveTime = 0;
    private static final String PERSISTED_DATA_FILE_NAME = "last_pose.txt";
    private static final String TELEMETRY_LOG_FILE_NAME = "telemetry_log.csv";

    // Holds the current header in memory so we don't re‐read the file every iteration.
    private static List<String> cachedHeaderKeys = null;

    private void appendTelemetryLine(Map<String, String> data) {
        File file = AppUtil.getInstance().getSettingsFile(TELEMETRY_LOG_FILE_NAME);
        boolean fileExists = file.exists();

        // If we've never established our cachedHeaderKeys in this run, attempt to read from existing file
        if (cachedHeaderKeys == null && fileExists) {
            try (BufferedReader br = new BufferedReader(new FileReader(file))) {
                String headerLine = br.readLine(); // e.g. "timestamp,x,y,heading,..."
                if (headerLine != null) {
                    List<String> headerList = Arrays.asList(headerLine.split(","));
                    // If first item is "timestamp", the rest are keys
                    if (!headerList.isEmpty() && headerList.get(0).equals("timestamp")) {
                        cachedHeaderKeys = headerList.subList(1, headerList.size());
                    }
                }
            } catch (Exception e) {
                telemetry.log().add("Failed to read existing header: " + e.getMessage());
            }
        }

        // Convert current data's keys to a list (in insertion order, if LinkedHashMap)
        List<String> currentKeys = new ArrayList<>(data.keySet());

        // If we already have a cached header, compare with current keys
        if (cachedHeaderKeys != null) {
            if (!cachedHeaderKeys.equals(currentKeys)) {
                // Key mismatch => reset (delete) file & drop the cached header
                if (file.exists() && !file.delete()) {
                    telemetry.log().add("Failed to delete old telemetry file.");
                }
                fileExists = false;
                cachedHeaderKeys = null;
            }
        }

        // If file doesn't exist (either didn't before or we just deleted it), write the new header
        if (!fileExists) {
            try (FileWriter fw = new FileWriter(file, true)) {
                fw.write("timestamp");
                for (String key : currentKeys) fw.write("," + key);
                fw.write("\n");
                cachedHeaderKeys = currentKeys;
            } catch (Exception e) {
                telemetry.log().add("Failed to write new header: " + e.getMessage());
                return; // Quit without appending data row
            }
        }

        // Append one row of CSV data
        try (FileWriter fw = new FileWriter(file, true)) {
            long now = System.currentTimeMillis();
            fw.write(String.valueOf(now));
            for (String key : currentKeys) {
                fw.write("," + data.get(key));
            }
            fw.write("\n");
        } catch (Exception e) {
            telemetry.log().add("Failed to log telemetry: " + e.getMessage());
        }
    }

    private boolean debugMode = false;
    private final AllianceColor allianceColor;
    private final OpModeState startupState;
    private AutonomousPlan autonomousPlan = null;

    public DecodeOpMode(AllianceColor allianceColor, OpModeState startupState) {
        this(allianceColor, startupState, null, false);
    }

    public DecodeOpMode(AllianceColor allianceColor, OpModeState startupState, AutonomousPlan autonomousPlan) {
        this(allianceColor, startupState, autonomousPlan, false);
    }

    public DecodeOpMode(AllianceColor allianceColor, OpModeState startupState, boolean debugMode) {
        this(allianceColor, startupState, null, debugMode);
    }

    private DecodeOpMode(AllianceColor allianceColor, OpModeState startupState, AutonomousPlan autonomousPlan, boolean debugMode) {
        super();
        this.allianceColor = allianceColor;
        this.startupState = startupState;
        this.autonomousPlan = autonomousPlan;
        this.debugMode = debugMode;
    }

    private void savePersistedData(Pose2d pose, int appendageState, int armLTickPosition, int armRTickPosition, int extenderTickPosition) {
        long timestamp = System.currentTimeMillis();
        try {
            File file = AppUtil.getInstance().getSettingsFile(PERSISTED_DATA_FILE_NAME);
            String data = timestamp + "\n" + pose.getX() + "\n" + pose.getY() + "\n" + pose.getHeading() + "\n" + appendageState + "\n" + armLTickPosition + "\n" + armRTickPosition + "\n" + extenderTickPosition;
            ReadWriteFile.writeFile(file, data);
        } catch (Exception e) {
            telemetry.log().add("Failed to save: " + e.getMessage());
        }
    }

    private static class PersistedData {
        public Pose2d pose;
        public long timestamp;
    }

    private PersistedData readPersistedData(boolean deleteAfterRead) {
        PersistedData persistedData = null;
        try {
            File poseFile = AppUtil.getInstance().getSettingsFile(PERSISTED_DATA_FILE_NAME);
            if (poseFile.exists()) {
                String[] lines = ReadWriteFile.readFile(poseFile).split("\n");
                if (lines.length >= 8) {
                    long timestamp = Long.parseLong(lines[0]);
                    double x = Double.parseDouble(lines[1]);
                    double y = Double.parseDouble(lines[2]);
                    double heading = Double.parseDouble(lines[3]);
                    Pose2d pose = new Pose2d(x, y, heading);
                    persistedData = new PersistedData();
                    persistedData.pose = pose;
                    persistedData.timestamp = timestamp;
                }
                if (deleteAfterRead) {
                    boolean cleanedUp = poseFile.delete();
                }
            }
        } catch (Exception e) {
        }
        return persistedData;
    }

    private Pose2d computeTeleopInitialPose() {
        PersistedData persistedData = readPersistedData(false);
        long initTime = System.currentTimeMillis();
        boolean persistedDataIsValid = persistedData != null &&
                initTime - persistedData.timestamp < EXPIRY_INTERVAL_MS;
        if (debugMode || !persistedDataIsValid) {
            // Default to facing across the field; driver can flip which side is front during teleop.
            return new Pose2d(0, 0, Math.toRadians(180));
        }
        return persistedData.pose;
    }

    @Override
    public void runOpMode() {
        // Hooks up telemetry data to the dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Pose2d initPose = autonomousPlan != null
                ? DecodeRobotControl.getStartPoseForPlan(allianceColor, autonomousPlan)
                : computeTeleopInitialPose();
        DecodeRobotControl terabytes = new DecodeRobotControl(
                allianceColor,
                initPose,
                gamepad1,
                gamepad2,
                hardwareMap,
                debugMode);
        dashboard.startCameraStream(terabytes.visionPortal, 15);
        if (autonomousPlan != null) {
            terabytes.autonomousInit(autonomousPlan, initPose);
        } else {
            terabytes.teleopInit(initPose);
        }

        terabytes.initializeMechanicalBlocking();

        while (!isStarted() && !isStopRequested()) {
            if (autonomousPlan != null) {
                // Keep the autonomous start pose locked in case Pinpoint resets finish late.
                terabytes.refreshPoseEstimate();
                terabytes.forcePoseEstimate(initPose);
            }
            dashboard.sendTelemetryPacket(terabytes.getTelemetryPacket());
            idle();
        }

        waitForStart();

        if (!isStopRequested() && autonomousPlan == null) {
            PersistedData persistedData = readPersistedData(true);
            long initTime = System.currentTimeMillis();
            boolean persistedDataIsValid = persistedData != null &&
                    initTime - persistedData.timestamp < EXPIRY_INTERVAL_MS;
            if (!debugMode && persistedDataIsValid) {
                terabytes.teleopInit(persistedData.pose);
            }
        }

        if (!isStopRequested()) {
            terabytes.startup(startupState);
        }

        lastSaveTime = System.currentTimeMillis();

        while (!isStopRequested() && terabytes.evaluate()) {
            long currentTime = System.currentTimeMillis();
            if (autonomousPlan != null && currentTime - lastSaveTime >= SAVE_INTERVAL_MS && opModeIsActive()) {
                Pose2d poseToPersist = terabytes.getLatestPoseEstimate();
                if (poseToPersist != null) {
                    savePersistedData(poseToPersist, 0, 0, 0, 0);
                }
                lastSaveTime = currentTime;
            }
            appendTelemetryLine(terabytes.getLogData());
            dashboard.sendTelemetryPacket(terabytes.getTelemetryPacket());
        }

        if (autonomousPlan != null) {
            Pose2d poseToPersist = terabytes.getLatestPoseEstimate();
            if (poseToPersist != null) {
                savePersistedData(poseToPersist, 0, 0, 0, 0);
            }
        }

        terabytes.shutDown();
    }
}
