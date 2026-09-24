package org.firstinspires.ftc.teamcode.command;

import org.firstinspires.ftc.teamcode.control.RobotActions;

/** One nonblocking robot behavior with explicit lifecycle and outcome. */
public interface RobotCommand {
    String getName();
    void start(RobotActions robot);
    CommandStatus update(RobotActions robot, double dtSeconds);
    void stop(RobotActions robot, boolean interrupted);
}
