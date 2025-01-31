package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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

public abstract class TerabytesOpMode extends LinearOpMode {

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
        public int appendageStateOrdinal;
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
                if (lines.length >= 8) {
                    long timestamp = Long.parseLong(lines[0]);
                    double x = Double.parseDouble(lines[1]);
                    double y = Double.parseDouble(lines[2]);
                    double heading = Double.parseDouble(lines[3]);
                    int appendageStateOrdinal = Integer.parseInt(lines[4]);
                    int armLTicks = Integer.parseInt(lines[5]);
                    int armRTicks = Integer.parseInt(lines[6]);
                    int extenderTicks = Integer.parseInt(lines[7]);
                    Pose2d pose = new Pose2d(x, y, heading);
                    persistedData = new PersistedData();
                    persistedData.pose = pose;
                    persistedData.appendageStateOrdinal = appendageStateOrdinal;
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
        dashboard.startCameraStream(terabytes.visionPortal, 15);
        if (autonomousPlan != null) {
            terabytes.autonomousInit(autonomousPlan);
        } else {
            PersistedData persistedData = readAndDeleteLastPersistedData();
            long initTime = System.currentTimeMillis();
            boolean persistedDataIsValid = persistedData != null &&
                    initTime - persistedData.timestamp < EXPIRY_INTERVAL_MS &&
                    persistedData.appendageStateOrdinal != -1 &&
                    persistedData.appendageStateOrdinal < AppendageControlState.values().length;
            if (debugMode || !persistedDataIsValid) {
                terabytes.teleopInit(new Pose2d(0, 0, Math.toRadians(180) + allianceColor.OperatorHeadingOffset));
            } else {
                AppendageControlState appendageControlState = AppendageControlState.values()[persistedData.appendageStateOrdinal];
                terabytes.teleopInit(
                        persistedData.pose,
                        appendageControlState,
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
            if (currentTime - lastSaveTime >= SAVE_INTERVAL_MS && opModeIsActive()) {
                savePersistedData(
                        terabytes.getLatestPoseEstimate(),
                        terabytes.getAppendageControlStateInteger(),
                        terabytes.getArmLTickPosition(),
                        terabytes.getArmRTickPosition(),
                        terabytes.getExtenderTickPosition());
                lastSaveTime = currentTime;
            }
            appendTelemetryLine(terabytes.getLogData());
            dashboard.sendTelemetryPacket(terabytes.getTelemetryPacket());
        }

        terabytes.shutDown();
    }
}
