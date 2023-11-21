package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Random;

@Config
@Autonomous(group = "digital-foundry-demo")
public class RandomDrive extends LinearOpMode {

    public static int SEQUENCE_LENGTH = 128;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);

        Random random = new Random();

        waitForStart();

        if (isStopRequested()) return;

        for (int i = 0; i < SEQUENCE_LENGTH && opModeIsActive(); i++) {
            double forwardDistance = random.nextDouble() * 10.0; // Random distance up to 10 units
            double turnAngle = random.nextDouble() * 360.0 - 180.0; // Random angle between -180 and 180 degrees

            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .forward(forwardDistance)
                    .turn(Math.toRadians(turnAngle))
                    .build();

            drive.followTrajectorySequence(trajSeq);
        }
    }
}