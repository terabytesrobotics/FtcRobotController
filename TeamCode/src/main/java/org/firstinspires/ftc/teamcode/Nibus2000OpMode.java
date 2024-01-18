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
    private AlliancePose alliancePose = null;
    private NibusAutonomousPlan autonomousPlan = null;


    public Nibus2000OpMode(AllianceColor allianceColor, NibusState startupState) {
        super();
        this.allianceColor = allianceColor;
        this.startupState = startupState;
    }

    public Nibus2000OpMode(AllianceColor allianceColor, AlliancePose startPose, NibusState startupState, NibusAutonomousPlan autonomousPlan) {
        super();
        this.allianceColor = allianceColor;
        this.alliancePose = startPose;
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
    }
}
