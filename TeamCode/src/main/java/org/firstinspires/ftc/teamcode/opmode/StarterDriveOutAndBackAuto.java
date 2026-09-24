package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.autonomous.StarterAutoPlans;
import org.firstinspires.ftc.teamcode.command.CommandStatus;
import org.firstinspires.ftc.teamcode.command.SequentialCommandRunner;
import org.firstinspires.ftc.teamcode.util.LoopTimer;

@Autonomous(name = "Starter: Drive Out and Back", group = "Starter Bot")
public class StarterDriveOutAndBackAuto extends OpMode {
    private static final double MAX_LOOP_DT_SECONDS = 0.1;

    private final LoopTimer loopTimer = new LoopTimer(MAX_LOOP_DT_SECONDS);
    private Robot robot;
    private SequentialCommandRunner commandRunner;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(
                telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot(hardwareMap);

        Pose2D startPose = new Pose2D(
                DistanceUnit.MM, 0.0, 0.0,
                AngleUnit.DEGREES, 0.0);
        robot.setPose(startPose);
        commandRunner = StarterAutoPlans.driveOutAndBack(startPose);

        telemetry.addLine("Raise the robot and verify drive direction before field testing");
        telemetry.addLine("Plan: wait, drive forward 300 mm, wait, return to start");
    }

    @Override
    public void init_loop() {
        robot.updateSensors();
        publishTelemetry();
    }

    @Override
    public void start() {
        loopTimer.reset();
        commandRunner.start();
    }

    @Override
    public void loop() {
        double dtSeconds = loopTimer.nextSeconds();
        robot.updateSensors();
        commandRunner.update(robot, dtSeconds);
        publishTelemetry();
    }

    @Override
    public void stop() {
        if (commandRunner.getStatus() == CommandStatus.RUNNING) {
            commandRunner.cancel(robot);
        } else {
            robot.stop();
        }
    }

    private void publishTelemetry() {
        telemetry.addData("Sequence", commandRunner.getStatus());
        telemetry.addData("Command", "%d/%d %s",
                commandRunner.getCommandNumber(),
                commandRunner.getCommandCount(),
                commandRunner.getCurrentCommandName());
        telemetry.addData("Pose X/Y/heading", "%.1f / %.1f mm / %.1f deg",
                robot.getX(), robot.getY(), robot.getHeadingDeg());
    }
}
