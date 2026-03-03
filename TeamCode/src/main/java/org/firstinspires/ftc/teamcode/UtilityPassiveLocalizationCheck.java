package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileWriter;

@Config
@TeleOp(name = "Utility: Passive Localization Check", group = "Utility")
public class UtilityPassiveLocalizationCheck extends LinearOpMode {
    public static boolean APPLY_180_FRAME_ROTATION = false;
    public static double X_POD_OFFSET_MM = -120.0;
    public static double Y_POD_OFFSET_MM = 0.0;
    public static long LOG_PERIOD_MS = 100;
    public static double OVERLAY_HEADING_LEN_IN = 10.0;

    private static final String LOG_FILE_NAME = "passive_localization_log.csv";

    private boolean lastA = false;
    private long lastLogTimeMs = 0;

    private void writeLogHeader(File file) {
        try (FileWriter fw = new FileWriter(file, false)) {
            fw.write("timestamp_ms,raw_x_in,raw_y_in,raw_heading_deg,mapped_x_in,mapped_y_in,mapped_heading_deg,encoder_x,encoder_y,pinpoint_heading_rad\n");
        } catch (Exception ignored) {
        }
    }

    private void appendLogLine(
            File file,
            long nowMs,
            double rawX,
            double rawY,
            double rawHeadingDeg,
            double mappedX,
            double mappedY,
            double mappedHeadingDeg,
            double encoderX,
            double encoderY,
            double pinpointHeadingRad) {
        try (FileWriter fw = new FileWriter(file, true)) {
            fw.write(nowMs + "," + rawX + "," + rawY + "," + rawHeadingDeg + ","
                    + mappedX + "," + mappedY + "," + mappedHeadingDeg + ","
                    + encoderX + "," + encoderY + "," + pinpointHeadingRad + "\n");
        } catch (Exception ignored) {
        }
    }

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setOffsets(X_POD_OFFSET_MM, Y_POD_OFFSET_MM, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pinpoint.resetPosAndIMU();

        File logFile = AppUtil.getInstance().getSettingsFile(LOG_FILE_NAME);
        writeLogHeader(logFile);

        telemetry.addLine("Passive localization check");
        telemetry.addLine("Push/rotate robot by hand");
        telemetry.addLine("A = reset pose + IMU");
        telemetry.addData("LogFile", LOG_FILE_NAME);
        telemetry.update();

        waitForStart();

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        while (opModeIsActive() && !isStopRequested()) {
            boolean aPressed = gamepad1.a;
            if (aPressed && !lastA) {
                pinpoint.resetPosAndIMU();
            }
            lastA = aPressed;

            pinpoint.update();
            Pose2D raw = pinpoint.getPosition();

            double rawX = raw.getX(DistanceUnit.INCH);
            double rawY = raw.getY(DistanceUnit.INCH);
            double rawHeading = raw.getHeading(AngleUnit.RADIANS);

            double mappedX = rawX;
            double mappedY = rawY;
            double mappedHeading = rawHeading;
            if (APPLY_180_FRAME_ROTATION) {
                mappedX = -rawX;
                mappedY = -rawY;
                mappedHeading = Angle.norm(rawHeading + Math.PI);
            }

            double rawHeadingDeg = Math.toDegrees(rawHeading);
            double mappedHeadingDeg = Math.toDegrees(mappedHeading);

            telemetry.addData("Apply180", APPLY_180_FRAME_ROTATION);
            telemetry.addData("RawX(in)", rawX);
            telemetry.addData("RawY(in)", rawY);
            telemetry.addData("RawHeading(deg)", rawHeadingDeg);
            telemetry.addData("MappedX(in)", mappedX);
            telemetry.addData("MappedY(in)", mappedY);
            telemetry.addData("MappedHeading(deg)", mappedHeadingDeg);
            telemetry.addData("PinpointEncoderX", pinpoint.getEncoderX());
            telemetry.addData("PinpointEncoderY", pinpoint.getEncoderY());
            telemetry.addData("PinpointHeading(rad)", pinpoint.getHeading(org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit.RADIANS));
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Apply180", APPLY_180_FRAME_ROTATION);
            packet.put("RawX(in)", rawX);
            packet.put("RawY(in)", rawY);
            packet.put("RawHeading(deg)", rawHeadingDeg);
            packet.put("MappedX(in)", mappedX);
            packet.put("MappedY(in)", mappedY);
            packet.put("MappedHeading(deg)", mappedHeadingDeg);
            packet.put("PinpointEncoderX", pinpoint.getEncoderX());
            packet.put("PinpointEncoderY", pinpoint.getEncoderY());
            packet.put("PinpointHeading(rad)", pinpoint.getHeading(org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit.RADIANS));

            Canvas overlay = packet.fieldOverlay();
            overlay.setStroke("#808080");
            overlay.strokeCircle(0, 0, 2);

            double rawX2 = rawX + OVERLAY_HEADING_LEN_IN * Math.cos(rawHeading);
            double rawY2 = rawY + OVERLAY_HEADING_LEN_IN * Math.sin(rawHeading);
            overlay.setStroke("#1E90FF");
            overlay.strokeCircle(rawX, rawY, 3);
            overlay.strokeLine(rawX, rawY, rawX2, rawY2);

            double mappedX2 = mappedX + OVERLAY_HEADING_LEN_IN * Math.cos(mappedHeading);
            double mappedY2 = mappedY + OVERLAY_HEADING_LEN_IN * Math.sin(mappedHeading);
            overlay.setStroke("#FF4500");
            overlay.strokeCircle(mappedX, mappedY, 4);
            overlay.strokeLine(mappedX, mappedY, mappedX2, mappedY2);

            dashboard.sendTelemetryPacket(packet);

            long nowMs = (long) runtime.milliseconds();
            if (nowMs - lastLogTimeMs >= LOG_PERIOD_MS) {
                appendLogLine(
                        logFile,
                        nowMs,
                        rawX,
                        rawY,
                        rawHeadingDeg,
                        mappedX,
                        mappedY,
                        mappedHeadingDeg,
                        pinpoint.getEncoderX(),
                        pinpoint.getEncoderY(),
                        pinpoint.getHeading(org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit.RADIANS)
                );
                lastLogTimeMs = nowMs;
            }
        }
    }
}
