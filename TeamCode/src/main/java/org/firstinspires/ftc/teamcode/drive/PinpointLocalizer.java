package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Road Runner Localizer backed by the goBILDA Pinpoint (2-wheel odometry + IMU).
 * Reads the fused pose from the Pinpoint each update and mirrors setPoseEstimate
 * back into the sensor so AprilTags or other corrections stay aligned.
 */
public class PinpointLocalizer implements Localizer {
    private final GoBildaPinpointDriver pinpoint;
    private Pose2d poseEstimate = new Pose2d();

    public PinpointLocalizer(GoBildaPinpointDriver pinpoint) {
        this.pinpoint = pinpoint;
    }

    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        return poseEstimate;
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose) {
        poseEstimate = pose;
        // Keep Pinpoint's internal pose aligned with RR's pose.
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, pose.getX(), pose.getY(),
                AngleUnit.RADIANS, pose.getHeading()));
    }

    @Override
    public Pose2d getPoseVelocity() {
        // Pinpoint API does not expose velocity directly.
        return null;
    }

    @Override
    public void update() {
        pinpoint.update();
        Pose2D pos = pinpoint.getPosition();
        poseEstimate = new Pose2d(
                pos.getX(DistanceUnit.INCH),
                pos.getY(DistanceUnit.INCH),
                pos.getHeading(AngleUnit.RADIANS)
        );
    }
}
