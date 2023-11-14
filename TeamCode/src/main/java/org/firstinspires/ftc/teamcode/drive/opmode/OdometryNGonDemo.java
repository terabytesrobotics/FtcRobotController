package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="OdometryNGonDemo", group="Linear Opmode")
public class OdometryNGonDemo extends OpMode {

    private SampleMecanumDrive drive;
    private Pose2d currentPose;
    private List<Pose2d> nGonVertices;
    private int currentVertexIndex;
    private int n; // Number of sides of the n-gon
    private int step; // Step length, coprime with n

    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        currentPose = new Pose2d(0, 0, Math.toRadians(0)); // Start position
        drive.setPoseEstimate(currentPose);

        n = 6; // Example: Hexagon
        step = 5; // Example step length coprime with 6
        nGonVertices = generateNGonVertices(n, 2); // 2 ft radius n-gon
        currentVertexIndex = 0;
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            currentVertexIndex = (currentVertexIndex + step) % nGonVertices.size();
            driveToPosition(nGonVertices.get(currentVertexIndex));
        }
    }

    private void driveToPosition(Pose2d targetPose) {
        Trajectory trajectory = drive.trajectoryBuilder(currentPose)
            .splineTo(targetPose)
            .build();
        drive.followTrajectory(trajectory);
        currentPose = targetPose;
    }

    private List<Pose2d> generateNGonVertices(int sides, double radius) {
        List<Pose2d> vertices = new ArrayList<>();
        for (int i = 0; i < sides; i++) {
            double angle = 2 * Math.PI * i / sides;
            double x = radius * Math.cos(angle);
            double y = radius * Math.sin(angle);
            vertices.add(new Pose2d(x, y, angle));
        }
        return vertices;
    }
}
