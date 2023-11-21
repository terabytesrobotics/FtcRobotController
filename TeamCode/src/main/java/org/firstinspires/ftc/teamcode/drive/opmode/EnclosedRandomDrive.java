package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Random;

@Config
@Autonomous(group = "digital-foundry-demo")
public class EnclosedRandomDrive extends LinearOpMode {
    private static final double ENCOLSURE_SIZE = 72; // assuming a square enclosure
    private static final int SEQUENCE_COUNT = 5; // number of splines
    private static final double CLEARANCE = 13; // clearance in inches

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Random random = new Random();

        waitForStart();

        if (isStopRequested()) return;

        Pose2d startPose = new Pose2d(0, 0, 0); // starting at the center
        drive.setPoseEstimate(startPose);
        for (int i = 0; i < SEQUENCE_COUNT; i++) {
            Pose2d randomPose = getRandomPoseWithinEnclosure(random);
            Trajectory traj = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .splineToSplineHeading(randomPose, randomPose.getHeading())
                    .build();

            drive.followTrajectory(traj);
        }
    }

    private Pose2d getRandomPoseWithinEnclosure(Random random) {
        double x = random.nextDouble() * (ENCOLSURE_SIZE - 2 * CLEARANCE) - (ENCOLSURE_SIZE / 2 - CLEARANCE);
        double y = random.nextDouble() * (ENCOLSURE_SIZE - 2 * CLEARANCE) - (ENCOLSURE_SIZE / 2 - CLEARANCE);
        double heading = random.nextDouble() * 2 * Math.PI; // random heading between 0 and 360 degrees
        return new Pose2d(x, y, heading);
    }
}