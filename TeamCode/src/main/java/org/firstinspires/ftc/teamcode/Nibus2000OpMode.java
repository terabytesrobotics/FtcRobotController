package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.AlliancePose;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;

public abstract class Nibus2000OpMode extends LinearOpMode {

    private final MultipleTelemetry multipleTelemetry;
    private final AllianceColor allianceColor;
    private final Pose2d startPose;
    private final NibusState startupState;

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
        try (FileOutputStream fos = new FileOutputStream(POSE_FILE);
             ObjectOutputStream oos = new ObjectOutputStream(fos)) {
            oos.writeObject(pose);
        } catch (IOException e) {
            multipleTelemetry.addData("Error", "Failed to save pose: " + e.getMessage());
            multipleTelemetry.update();
        }
    }

    private Pose2d RecoverPoseEstimate() {
        File file = new File(POSE_FILE);
        if (file.exists()) {
            try (FileInputStream fis = new FileInputStream(file);
                 ObjectInputStream ois = new ObjectInputStream(fis)) {
                return (Pose2d) ois.readObject();
            } catch (IOException | ClassNotFoundException e) {
                multipleTelemetry.addData("Error", "Failed to recover pose: " + e.getMessage());
                multipleTelemetry.update();
            }
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
                multipleTelemetry);
        nibus.init(startPose);
        waitForStart();
        nibus.startup(startupState);
        while (!isStopRequested()) {
            nibus.evaluate();
        }
        SavePoseEstimate(nibus.getPoseEstimate());
    }
}
