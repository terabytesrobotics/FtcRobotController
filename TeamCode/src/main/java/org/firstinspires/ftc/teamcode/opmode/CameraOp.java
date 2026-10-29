package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.Robot;

import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Camera", group = "Debug")
public class CameraOp extends OpMode {
    Robot robot;
    FtcDashboard dashboard;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.setMsTransmissionInterval(50);
        robot = new Robot(hardwareMap);
        dashboard.setImageQuality(40);
        dashboard.startCameraStream(robot.getLimelight(), 8);
    }

    @Override
    public void loop() {
        robot.update();

        LLResult result = robot.getLatestLimelightResult();
        boolean valid = result != null && result.isValid();
        telemetry.addData("vision.valid", valid);

        if (valid) {
            List<LLResultTypes.FiducialResult> detections = result.getFiducialResults();
            telemetry.addData("vision.tagCount", detections.size());
            telemetry.addData("vision.captureLatencyMs", result.getCaptureLatency());
            telemetry.addData("vision.targetingLatencyMs", result.getTargetingLatency());

            for (int i = 0; i < detections.size(); i++) {
                LLResultTypes.FiducialResult detection = detections.get(i);
                String prefix = "vision.tag" + i + ".";
                telemetry.addData(prefix + "id", detection.getFiducialId());
                telemetry.addData(prefix + "txDeg", detection.getTargetXDegrees());
                telemetry.addData(prefix + "tyDeg", detection.getTargetYDegrees());
                telemetry.addData(prefix + "area", detection.getTargetArea());
            }
        } else {
            telemetry.addData("vision.tagCount", 0);
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        dashboard.stopCameraStream();
        robot.getLimelight().stop();
    }
}
