package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@TeleOp(group = "drive")
public class SplineStateMachineTeleOp extends LinearOpMode {
    private static final double DESTINATION_DISTANCE = 60; // adjust as needed

    private enum State {
        START, LEFT_PATH, CENTER_PATH, RIGHT_PATH, RETURN_TO_START, DESTINATION
    }

    private State currentState = State.START;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPosition = new Pose2d(0, 0, 0);
        Pose2d destination = new Pose2d(DESTINATION_DISTANCE, 0, 0);
        Pose2d leftJoin = new Pose2d(DESTINATION_DISTANCE / 3, -15, 0); // adjust as needed
        Pose2d rightJoin = new Pose2d(DESTINATION_DISTANCE / 3, 15, 0); // adjust as needed
        Pose2d centerJoin = new Pose2d(DESTINATION_DISTANCE / 3, 0, 0); // adjust as needed

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) currentState = State.RETURN_TO_START;
            if (gamepad1.x) currentState = State.LEFT_PATH;
            if (gamepad1.y) currentState = State.CENTER_PATH;
            if (gamepad1.b) currentState = State.RIGHT_PATH;

            switch (currentState) {
                case START:
                    // Robot is at start position
                    break;

                case LEFT_PATH:
                    executePath(drive, startPosition, leftJoin, destination);
                    currentState = State.DESTINATION;
                    break;

                case CENTER_PATH:
                    executePath(drive, startPosition, centerJoin, destination);
                    currentState = State.DESTINATION;
                    break;

                case RIGHT_PATH:
                    executePath(drive, startPosition, rightJoin, destination);
                    currentState = State.DESTINATION;
                    break;

                case RETURN_TO_START:
                    executeReturnPath(drive, startPosition);
                    currentState = State.START;
                    break;
            }

            // Other teleop code here (if any)
        }
    }

    private void executePath(SampleMecanumDrive drive, Pose2d start, Pose2d join, Pose2d end) {
        Trajectory toJoin = drive.trajectoryBuilder(start)
                .splineToSplineHeading(join, join.getHeading())
                .build();
        Trajectory toEnd = drive.trajectoryBuilder(toJoin.end())
                .splineToSplineHeading(end, end.getHeading())
                .build();

        drive.followTrajectory(toJoin);
        drive.followTrajectory(toEnd);
    }

    private void executeReturnPath(SampleMecanumDrive drive, Pose2d start) {
        Trajectory toStart = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineToSplineHeading(start, start.getHeading())
                .build();
        drive.followTrajectory(toStart);
    }
}