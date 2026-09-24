package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.control.DriveSignal;
import org.firstinspires.ftc.teamcode.control.TeleopController;
import org.firstinspires.ftc.teamcode.control.WheelPowers;
import org.firstinspires.ftc.teamcode.dashboard.DashboardField;
import org.firstinspires.ftc.teamcode.util.LoopTimer;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "Starter Bot")
public class TeleOp extends OpMode {
    private static final double DEADBAND = 0.05;
    private static final double MAX_LOOP_DT_SECONDS = 0.1;
    private static final double MM_PER_INCH = 25.4;
    private static final double ROBOT_RADIUS_INCHES = 9.0;

    private final LoopTimer loopTimer = new LoopTimer(MAX_LOOP_DT_SECONDS);
    private FtcDashboard dashboard;
    private Robot robot;
    private TeleopController controller;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        robot = new Robot(hardwareMap);
        controller = new TeleopController(gamepad1, DEADBAND);
        telemetry.addLine("Shared Robot teleop initialized");
        telemetry.addLine("Left bumper: precision | Right bumper: full speed");
        sendDashboardTelemetry();
    }

    @Override
    public void init_loop() {
        robot.updateSensors();
        sendDashboardTelemetry();
    }

    @Override
    public void start() {
        loopTimer.reset();
        controller.start(robot);
    }

    @Override
    public void loop() {
        double dtSeconds = loopTimer.nextSeconds();
        robot.updateSensors();
        controller.update(robot, dtSeconds);

        DriveSignal signal = controller.getLastDriveSignal();
        WheelPowers wheels = controller.getLastWheelPowers();
        telemetry.addData("Pose X/Y/heading", "%.1f / %.1f mm / %.1f deg",
                robot.getX(), robot.getY(), robot.getHeadingDeg());
        telemetry.addData("Drive forward/left/CCW", "%.2f / %.2f / %.2f",
                signal.getForward(), signal.getLeft(), signal.getCounterclockwise());
        telemetry.addData("Wheels FL/FR/BL/BR", "%.2f / %.2f / %.2f / %.2f",
                wheels.getFrontLeft(), wheels.getFrontRight(),
                wheels.getBackLeft(), wheels.getBackRight());
        telemetry.addData("Collector", "%.2f", controller.getLastCollectorPower());
        telemetry.addData("Speed limit", "%.0f%%", controller.getLastSpeed() * 100.0);
        sendDashboardTelemetry();
    }

    @Override
    public void stop() {
        robot.stop();
    }

    private void sendDashboardTelemetry() {
        DriveSignal signal = controller.getLastDriveSignal();
        WheelPowers wheels = controller.getLastWheelPowers();

        TelemetryPacket packet = new TelemetryPacket(false);
        packet.put("Collector", String.format("%.2f", controller.getLastCollectorPower()));
        packet.put("Drive forward/left/CCW", String.format(
                "%.2f / %.2f / %.2f",
                signal.getForward(), signal.getLeft(), signal.getCounterclockwise()));
        packet.put("Pose X/Y/heading", String.format(
                "%.1f / %.1f mm / %.1f deg",
                robot.getX(), robot.getY(), robot.getHeadingDeg()));
        packet.put("Speed limit", String.format("%.0f%%", controller.getLastSpeed() * 100.0));
        packet.put("Wheels FL/FR/BL/BR", String.format(
                "%.2f / %.2f / %.2f / %.2f",
                wheels.getFrontLeft(), wheels.getFrontRight(),
                wheels.getBackLeft(), wheels.getBackRight()));

        Canvas field = packet.fieldOverlay();
        DashboardField.drawBackground(field);
        DashboardField.drawPose(
                field,
                robot.getX() / MM_PER_INCH,
                robot.getY() / MM_PER_INCH,
                robot.getHeadingRad(),
                ROBOT_RADIUS_INCHES,
                "#2196F3");
        dashboard.sendTelemetryPacket(packet);
    }
}
