package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name = "Test: Limelight 3A", group = "Debug")
public class LimelightDiagnosticOp extends OpMode {
    private static final String LIMELIGHT_NAME = "limelight";

    // Select the pipeline from Dashboard. A change is sent only once, not every loop.
    public static int pipelineIndex = 0;

    private Limelight3A limelight;
    private int activePipelineIndex = -1;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(
                telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(50);

        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        switchPipelineIfNeeded();
        limelight.start();

        telemetry.addLine("Limelight polling started");
        telemetry.addLine("Change pipelineIndex from FTC Dashboard if needed");
    }

    @Override
    public void loop() {
        switchPipelineIfNeeded();

        LLStatus status = limelight.getStatus();
        telemetry.addData("Name", status.getName());
        telemetry.addData("Temperature", "%.1f C", status.getTemp());
        telemetry.addData("CPU", "%.1f%%", status.getCpu());
        telemetry.addData("FPS", "%.0f", status.getFps());
        telemetry.addData("Pipeline", "%d (%s)",
                status.getPipelineIndex(), status.getPipelineType());

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            telemetry.addData("Target valid", true);
            telemetry.addData("tx / ty", "%.2f / %.2f deg", result.getTx(), result.getTy());
            telemetry.addData("Latency", "capture %.1f + target %.1f + parse %.1f ms",
                    result.getCaptureLatency(),
                    result.getTargetingLatency(),
                    result.getParseLatency());
            telemetry.addData("Fiducials", result.getFiducialResults().size());
            telemetry.addData("Detections", result.getDetectorResults().size());
            telemetry.addData("Classifiers", result.getClassifierResults().size());
            telemetry.addData("Colors", result.getColorResults().size());
            telemetry.addData("Barcodes", result.getBarcodeResults().size());
            telemetry.addData("Bot pose", result.getBotpose());
        } else {
            telemetry.addData("Target valid", false);
        }
    }

    @Override
    public void stop() {
        limelight.stop();
    }

    private void switchPipelineIfNeeded() {
        if (pipelineIndex != activePipelineIndex) {
            if (limelight.pipelineSwitch(pipelineIndex)) {
                activePipelineIndex = pipelineIndex;
            } else {
                telemetry.addData("Pipeline error", "Could not select %d", pipelineIndex);
            }
        }
    }
}
