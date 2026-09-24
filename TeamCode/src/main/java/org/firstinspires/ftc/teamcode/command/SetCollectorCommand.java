package org.firstinspires.ftc.teamcode.command;

import org.firstinspires.ftc.teamcode.control.RobotActions;
import org.firstinspires.ftc.teamcode.subsystem.CollectorMode;

public final class SetCollectorCommand implements RobotCommand {
    private final CollectorMode mode;

    private SetCollectorCommand(CollectorMode mode) {
        this.mode = mode;
    }

    public static SetCollectorCommand to(CollectorMode mode) {
        if (mode == null) {
            throw new IllegalArgumentException("mode must not be null");
        }
        return new SetCollectorCommand(mode);
    }

    @Override
    public String getName() {
        return "Collector(" + mode + ")";
    }

    @Override
    public void start(RobotActions robot) {
        robot.setCollectorMode(mode);
    }

    @Override
    public CommandStatus update(RobotActions robot, double dtSeconds) {
        return CommandStatus.SUCCEEDED;
    }

    @Override
    public void stop(RobotActions robot, boolean interrupted) {
        if (interrupted) {
            robot.setCollectorMode(CollectorMode.OFF);
        }
    }
}
