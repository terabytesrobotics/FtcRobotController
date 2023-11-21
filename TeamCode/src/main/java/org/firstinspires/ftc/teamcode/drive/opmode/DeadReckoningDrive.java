package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous(group = "digital-foundry-demo")
public class DeadReckoningDrive extends LinearOpMode {
    private static final int NUM_SQUARES = 3; // You can change this value

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        for (int square = 0; square < NUM_SQUARES; square++) {
            // Drive forward
            drive.setWeightedDrivePower(new Pose2d(0.5, 0, 0));
            sleep(1000);
            drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
            sleep(250);

            // Strafe right
            drive.setWeightedDrivePower(new Pose2d(0, 0.5, 0));
            sleep(1000);
            drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
            sleep(250);

            // Drive backward
            drive.setWeightedDrivePower(new Pose2d(-0.5, 0, 0));
            sleep(1000);
            drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
            sleep(250);

            // Strafe left
            drive.setWeightedDrivePower(new Pose2d(0, -0.5, 0));
            sleep(1000);
            drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
            sleep(250);
        }

        // Stop the robot
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
    }
}