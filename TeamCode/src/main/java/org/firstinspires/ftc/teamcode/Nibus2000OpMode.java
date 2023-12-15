package org.firstinspires.ftc.teamcode;

import android.app.Application;
import android.content.Context;

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

public abstract class Nibus2000OpMode extends LinearOpMode {

    private final Telemetry multipleTelemetry;
    private final AllianceColor allianceColor;
    private final Pose2d startPose;
    private final NibusState startupState;
    protected final Context context = AppUtil.getDefContext();

    public Nibus2000OpMode(AllianceColor allianceColor, NibusState startupState) {
        super();
        this.multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.allianceColor = allianceColor;
        this.startPose = RecoverPoseEstimate();
        this.startupState = startupState;
    }

    public Nibus2000OpMode(AllianceColor allianceColor, AlliancePose startPose, NibusState startupState) {
        super();
        this.multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.allianceColor = allianceColor;
        this.startPose = allianceColor.getAbsoluteFieldPose(startPose);
        this.startupState = startupState;
    }

    private static final String POSE_FILE = "PoseData.tmp";

    private void SavePoseEstimate(Pose2d pose) {
        JSONObject poseJson = new JSONObject();
        try {
            poseJson.put("x", pose.getX());
            poseJson.put("y", pose.getY());
            poseJson.put("heading", pose.getHeading());
        } catch (JSONException e) {
            telemetry.addData("Error", "Failed to save pose json: " + e.getMessage());
            telemetry.update();
        }

        try (FileOutputStream fos = context.openFileOutput(POSE_FILE, Context.MODE_PRIVATE)) {
            fos.write(poseJson.toString().getBytes());
        } catch (IOException e) {
            telemetry.addData("Error", "Failed to save pose: " + e.getMessage());
            telemetry.update();
        }
    }

    private Pose2d RecoverPoseEstimate() {
        try (FileInputStream fis = context.openFileInput(POSE_FILE);
             InputStreamReader isr = new InputStreamReader(fis);
             BufferedReader reader = new BufferedReader(isr)) {

            StringBuilder stringBuilder = new StringBuilder();
            String line;
            while ((line = reader.readLine()) != null) {
                stringBuilder.append(line);
            }

            JSONObject poseJson = new JSONObject(stringBuilder.toString());
            return new Pose2d(
                    poseJson.getDouble("x"),
                    poseJson.getDouble("y"),
                    poseJson.getDouble("heading")
            );
        } catch (IOException | JSONException e) {
            telemetry.addData("Error", "Failed to recover pose: " + e.getMessage());
            telemetry.update();
        }
        return new Pose2d();
    }


    @Override
    public void runOpMode() {
        Nibus2000 nibus = new Nibus2000(
                allianceColor,
                gamepad1,
                gamepad2,
                hardwareMap,
                telemetry);
        nibus.init(startPose);
        waitForStart();
        nibus.startup(startupState);
        while (!isStopRequested() && !nibus.evaluate()) { }
        SavePoseEstimate(nibus.getPoseEstimate());
    }
}
