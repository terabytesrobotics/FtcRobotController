package org.firstinspires.ftc.teamcode.command;

import org.firstinspires.ftc.teamcode.control.RobotActions;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/** Minimal sequential scheduler used by autonomous without coupling plans to an FTC OpMode. */
public final class SequentialCommandRunner {
    private final List<RobotCommand> commands;
    private int commandIndex;
    private boolean currentCommandStarted;
    private CommandStatus status = CommandStatus.NOT_STARTED;

    public SequentialCommandRunner(List<RobotCommand> commands) {
        this.commands = new ArrayList<>(commands);
    }

    public static SequentialCommandRunner sequence(RobotCommand... commands) {
        return new SequentialCommandRunner(Arrays.asList(commands));
    }

    public void start() {
        commandIndex = 0;
        currentCommandStarted = false;
        status = commands.isEmpty() ? CommandStatus.SUCCEEDED : CommandStatus.RUNNING;
    }

    public CommandStatus update(RobotActions robot, double dtSeconds) {
        if (status != CommandStatus.RUNNING) {
            return status;
        }

        RobotCommand command = commands.get(commandIndex);
        if (!currentCommandStarted) {
            command.start(robot);
            currentCommandStarted = true;
        }

        CommandStatus commandStatus = command.update(robot, dtSeconds);
        if (commandStatus == CommandStatus.RUNNING) {
            return status;
        }

        command.stop(robot, false);
        currentCommandStarted = false;
        if (commandStatus != CommandStatus.SUCCEEDED) {
            status = commandStatus;
            robot.stopDrive();
            return status;
        }

        commandIndex++;
        if (commandIndex >= commands.size()) {
            status = CommandStatus.SUCCEEDED;
            robot.stopDrive();
        }
        return status;
    }

    public void cancel(RobotActions robot) {
        if (status == CommandStatus.RUNNING && currentCommandStarted) {
            commands.get(commandIndex).stop(robot, true);
        }
        robot.stop();
        status = CommandStatus.FAILED;
    }

    public CommandStatus getStatus() {
        return status;
    }

    public String getCurrentCommandName() {
        if (status != CommandStatus.RUNNING || commandIndex >= commands.size()) {
            return "none";
        }
        return commands.get(commandIndex).getName();
    }

    public int getCommandNumber() {
        return Math.min(commandIndex + 1, commands.size());
    }

    public int getCommandCount() {
        return commands.size();
    }
}
