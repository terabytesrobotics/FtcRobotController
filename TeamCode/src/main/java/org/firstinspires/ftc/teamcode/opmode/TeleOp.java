package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.control.DriveSignal;
import org.firstinspires.ftc.teamcode.control.TeleopController;
import org.firstinspires.ftc.teamcode.control.WheelPowers;
import org.firstinspires.ftc.teamcode.util.LoopTimer;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "Starter Bot")
public class TeleOp extends OpMode {
    private static final double DEADBAND = 0.05;
    private static final double MAX_LOOP_DT_SECONDS = 0.1;

    private final LoopTimer loopTimer = new LoopTimer(MAX_LOOP_DT_SECONDS);
    private Robot robot;
    private TeleopController controller;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(
                telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot(hardwareMap);
        controller = new TeleopController(gamepad1, DEADBAND);
        telemetry.addLine("Shared Robot teleop initialized");
        telemetry.addLine("Left bumper: precision | Right bumper: full speed");
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
    }

    @Override
    public void stop() {
        robot.stop();
    }
}
