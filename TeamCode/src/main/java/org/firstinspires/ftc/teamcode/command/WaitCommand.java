package org.firstinspires.ftc.teamcode.command;

import org.firstinspires.ftc.teamcode.control.RobotActions;

public final class WaitCommand implements RobotCommand {
    private final double durationSeconds;
    private double elapsedSeconds;

    private WaitCommand(double durationSeconds) {
        this.durationSeconds = durationSeconds;
    }

    public static WaitCommand milliseconds(double durationMs) {
        if (!Double.isFinite(durationMs) || durationMs < 0.0) {
            throw new IllegalArgumentException("Wait duration must be non-negative");
        }
        return new WaitCommand(durationMs / 1000.0);
    }

    @Override
    public String getName() {
        return String.format("Wait(%.0f ms)", durationSeconds * 1000.0);
    }

    @Override
    public void start(RobotActions robot) {
        elapsedSeconds = 0.0;
        robot.stopDrive();
    }

    @Override
    public CommandStatus update(RobotActions robot, double dtSeconds) {
        elapsedSeconds += dtSeconds;
        return elapsedSeconds >= durationSeconds
                ? CommandStatus.SUCCEEDED
                : CommandStatus.RUNNING;
    }

    @Override
    public void stop(RobotActions robot, boolean interrupted) {
        robot.stopDrive();
    }
}
