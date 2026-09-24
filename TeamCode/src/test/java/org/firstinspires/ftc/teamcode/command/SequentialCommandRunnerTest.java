package org.firstinspires.ftc.teamcode.command;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.control.DriveProfile;
import org.firstinspires.ftc.teamcode.control.DriveSignal;
import org.firstinspires.ftc.teamcode.control.MoveToResult;
import org.firstinspires.ftc.teamcode.control.RobotActions;
import org.firstinspires.ftc.teamcode.control.WheelPowers;
import org.firstinspires.ftc.teamcode.subsystem.CollectorMode;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class SequentialCommandRunnerTest {
    @Test
    public void advancesCommandsAndPreservesExplicitCollectorState() {
        FakeRobot robot = new FakeRobot();
        SequentialCommandRunner runner = SequentialCommandRunner.sequence(
                WaitCommand.milliseconds(100.0),
                SetCollectorCommand.to(CollectorMode.INTAKE));

        runner.start();
        assertEquals(CommandStatus.RUNNING, runner.update(robot, 0.04));
        assertEquals(CommandStatus.RUNNING, runner.update(robot, 0.07));
        assertEquals(CommandStatus.SUCCEEDED, runner.update(robot, 0.0));
        assertEquals(CollectorMode.INTAKE, robot.collectorMode);
    }

    @Test
    public void surfacesDriveTimeoutAsTerminalSequenceStatus() {
        FakeRobot robot = new FakeRobot();
        DriveProfile profile = DriveProfile.named("timeout-test")
                .settleTimeMs(100.0)
                .timeoutMs(100.0)
                .build();
        SequentialCommandRunner runner = SequentialCommandRunner.sequence(
                DriveToCommand.to(nullPose()).using(profile));

        runner.start();
        assertEquals(CommandStatus.RUNNING, runner.update(robot, 0.05));
        assertEquals(CommandStatus.TIMED_OUT, runner.update(robot, 0.06));
    }

    private static Pose2D nullPose() {
        return new Pose2D(
                org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM,
                0.0, 0.0,
                org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS,
                0.0);
    }

    private static final class FakeRobot implements RobotActions {
        private CollectorMode collectorMode = CollectorMode.OFF;

        @Override
        public MoveToResult moveTo(Pose2D targetPose, DriveProfile profile, double dtSeconds) {
            return new MoveToResult(
                    targetPose,
                    targetPose,
                    100.0,
                    0.0,
                    100.0,
                    0.0,
                    0.0,
                    DriveSignal.ZERO,
                    WheelPowers.ZERO,
                    false);
        }

        @Override
        public void resetMoveToControllers() {
        }

        @Override
        public void setCollectorMode(CollectorMode mode) {
            collectorMode = mode;
        }

        @Override
        public void stopDrive() {
        }

        @Override
        public void stop() {
            collectorMode = CollectorMode.OFF;
        }
    }
}
