package org.firstinspires.ftc.teamcode;

import android.app.Application;
import android.content.Context;
import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.fasterxml.jackson.core.io.JsonEOFException;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.AlliancePose;
import org.firstinspires.ftc.teamcode.util.BlueGrabberState;
import org.firstinspires.ftc.teamcode.util.CollectorState;
import org.firstinspires.ftc.teamcode.util.GreenGrabberState;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;

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
    protected final Context context = AppUtil.getDefContext();
    private AlliancePose alliancePose = null;
    private NibusSaveState saveState = null;


    public Nibus2000OpMode(AllianceColor allianceColor, NibusState startupState) {
        super();
        this.allianceColor = allianceColor;
        this.saveState = ReadNibusSaveState();
        this.startupState = startupState;
    }

    public Nibus2000OpMode(AllianceColor allianceColor, AlliancePose startPose, NibusState startupState) {
        super();
        this.allianceColor = allianceColor;
        this.alliancePose = startPose;
        this.startupState = startupState;
    }

    private static final String POSE_FILE = "PoseData.tmp";

    private void WriteNibusSaveState(
            Pose2d pose,
            int armPosition,
            int extenderPosition,
            CollectorState collectorState,
            BlueGrabberState blueGrabberState,
            GreenGrabberState greenGrabberState) {
        JSONObject json = new JSONObject();
        try {
            json.put("x", pose.getX());
            json.put("y", pose.getY());
            json.put("heading", pose.getHeading());
            json.put("armPosition", armPosition);
            json.put("extenderPosition", extenderPosition);
            json.put("collectorState", collectorState.name());
            json.put("blueGrabberState", blueGrabberState.name());
            json.put("greenGrabberState", greenGrabberState.name());
        } catch (JSONException e) {
            telemetry.addData("Error", "Failed to save pose json: " + e.getMessage());
            telemetry.update();
        }

        try (FileOutputStream fos = context.openFileOutput(POSE_FILE, Context.MODE_PRIVATE)) {
            String jsonString = json.toString();
            Log.d("nibusSaveState", jsonString);
            fos.write(jsonString.getBytes());
        } catch (IOException e) {
            telemetry.addData("Error", "Failed to save pose: " + e.getMessage());
            telemetry.update();
        }
    }

    private NibusSaveState ReadNibusSaveState() {
        try (FileInputStream fis = context.openFileInput(POSE_FILE);
             InputStreamReader isr = new InputStreamReader(fis);
             BufferedReader reader = new BufferedReader(isr)) {

            StringBuilder stringBuilder = new StringBuilder();
            String line;
            while ((line = reader.readLine()) != null) {
                stringBuilder.append(line);
            }

            String jsonString = stringBuilder.toString();
            Log.d("nibusSaveState", jsonString);
            JSONObject json = new JSONObject(stringBuilder.toString());
            Pose2d pose = new Pose2d(
                    json.getDouble("x"),
                    json.getDouble("y"),
                    json.getDouble("heading"));

            int armPosition = json.getInt("armPosition");
            int extenderPosition = json.getInt("extenderPosition");
            CollectorState collectorState = CollectorState.valueOf(json.getString("collectorState"));
            BlueGrabberState blueGrabberState = BlueGrabberState.valueOf(json.getString("blueGrabberState"));
            GreenGrabberState greenGrabberState = GreenGrabberState.valueOf(json.getString("greenGrabberState"));
            return new NibusSaveState(pose, armPosition, extenderPosition, collectorState, blueGrabberState, greenGrabberState);
        } catch (IOException | JSONException e) {
            telemetry.addData("Error", "Failed to recover pose: " + e.getMessage());
            telemetry.update();
        }
        return null;
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
            nibus.autonomousInit(alliancePose);
        } else {
            nibus.teleopInit(saveState);
        }
        waitForStart();
        nibus.startup(startupState);

        // TODO: need to get these before stop requested.
        int lastArmPosition = 0;
        int lastExtenderPosition = 0;
        while (!isStopRequested() && nibus.evaluate()) {
            lastArmPosition = nibus.getArmPosition();
            lastExtenderPosition = nibus.getExtenderPosition();
        }
        WriteNibusSaveState(
                nibus.getPoseEstimate(),
                lastArmPosition,
                lastExtenderPosition,
                nibus.getCollectorState(),
                nibus.getBlueGrabberState(),
                nibus.getGreenGrabberState());
    }
}
