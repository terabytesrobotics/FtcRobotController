package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "TogglePositionOpMode")
public class TogglePositionOpMode extends OpMode {
    private enum State {
        AT_START,
        MOVING_TO_END,
        AT_END,
        MOVING_TO_START
    }

    private SampleMecanumDrive drive;
    private State currentState = State.AT_START;
    private Trajectory trajectoryToEnd;
    private Trajectory trajectoryToStart;
    private Pose2d startPosition = new Pose2d(0, 0, 0);
    private Pose2d endPosition = new Pose2d(10, 0, 0); // Adjust as needed
    private AprilTagProcessor aprilTag; // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null; // Used to hold the data for a detected AprilTag

    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        trajectoryToEnd = drive.trajectoryBuilder(startPosition)
            .lineTo(endPosition.vec())
            .build();
        trajectoryToStart = drive.trajectoryBuilder(endPosition)
            .lineTo(startPosition.vec())
            .build();
    }

    @Override
    public void loop() {
        switch (currentState) {
            case AT_START:
                if (gamepad1.a) {
                    drive.followTrajectoryAsync(trajectoryToEnd);
                    currentState = State.MOVING_TO_END;
                }
                break;
            case MOVING_TO_END:
                if (!drive.isBusy()) {
                    currentState = State.AT_END;
                    // Trigger AprilTag detection here and adjust localization
                }
                break;
            case AT_END:
                if (gamepad1.a) {
                    drive.followTrajectoryAsync(trajectoryToStart);
                    currentState = State.MOVING_TO_START;
                }
                break;
            case MOVING_TO_START:
                if (!drive.isBusy()) {
                    currentState = State.AT_START;
                }
                break;
        }

        drive.update();
    }

    private void adjustLocalizationUsingAprilTag() {
      List<AprilTagDetection> detections = aprilTag.getDetections();
      if (!detections.isEmpty()) {
          // Example: averaging the positions of all detected tags
          double avgX = 0, avgY = 0, avgHeading = 0;
          for (AprilTagDetection detection : detections) {
              Pose2d pose = aprilTag.getPose(detection);
              avgX += pose.getX();
              avgY += pose.getY();
              avgHeading += pose.getHeading();
          }
          avgX /= detections.size();
          avgY /= detections.size();
          avgHeading /= detections.size();

          // Adjust robot localization here
          // Example: set the robot's position to the average position of the detections
          Pose2d correctedPose = new Pose2d(avgX, avgY, avgHeading);
          drive.setPoseEstimate(correctedPose);
      }
  }
}