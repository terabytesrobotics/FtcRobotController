package org.firstinspires.ftc.teamcode.command;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.control.DriveProfile;
import org.firstinspires.ftc.teamcode.control.MoveToResult;
import org.firstinspires.ftc.teamcode.control.RobotActions;

/** Drives to one field pose, requiring the configured continuous settle time before succeeding. */
public final class DriveToCommand implements RobotCommand {
    private final Pose2D targetPose;
    private final DriveProfile profile;
    private double elapsedSeconds;
    private double settledSeconds;
    private MoveToResult latestResult;

    private DriveToCommand(Pose2D targetPose, DriveProfile profile) {
        this.targetPose = targetPose;
        this.profile = profile;
    }

    public static ProfileSelection to(Pose2D targetPose) {
        return new ProfileSelection(targetPose);
    }

    @Override
    public String getName() {
        return "DriveTo(" + profile.getName() + ")";
    }

    @Override
    public void start(RobotActions robot) {
        elapsedSeconds = 0.0;
        settledSeconds = 0.0;
        latestResult = null;
        robot.resetMoveToControllers();
    }

    @Override
    public CommandStatus update(RobotActions robot, double dtSeconds) {
        elapsedSeconds += dtSeconds;
        latestResult = robot.moveTo(targetPose, profile, dtSeconds);
        settledSeconds = latestResult.isAtTarget() ? settledSeconds + dtSeconds : 0.0;

        if (settledSeconds >= profile.getSettleTimeSeconds()) {
            return CommandStatus.SUCCEEDED;
        }
        if (profile.getTimeoutSeconds() > 0.0
                && elapsedSeconds >= profile.getTimeoutSeconds()) {
            return CommandStatus.TIMED_OUT;
        }
        return CommandStatus.RUNNING;
    }

    @Override
    public void stop(RobotActions robot, boolean interrupted) {
        robot.stopDrive();
        robot.resetMoveToControllers();
    }

    public MoveToResult getLatestResult() {
        return latestResult;
    }

    public static final class ProfileSelection {
        private final Pose2D targetPose;

        private ProfileSelection(Pose2D targetPose) {
            if (targetPose == null) {
                throw new IllegalArgumentException("targetPose must not be null");
            }
            this.targetPose = targetPose;
        }

        public DriveToCommand using(DriveProfile profile) {
            if (profile == null) {
                throw new IllegalArgumentException("profile must not be null");
            }
            return new DriveToCommand(targetPose, profile);
        }
    }
}
