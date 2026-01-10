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
    private Pose2d poseEstimate;
    private double headingOffset = 0.0;
    private double translationOffsetX = 0.0;
    private double translationOffsetY = 0.0;

    public PinpointLocalizer(GoBildaPinpointDriver pinpoint, Pose2d initialPose) {
        this.pinpoint = pinpoint;
        // Seed with a known pose while letting Pinpoint come up asynchronously.
        Pose2d seedPose = initialPose != null ? initialPose : new Pose2d();
        setPoseEstimate(seedPose);
    }

    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        return poseEstimate;
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose) {
        // Track a rotation + translation offset between desired world pose and raw Pinpoint reading.
        Pose2D raw = pinpoint.getPosition();
        double rawHeading = raw.getHeading(AngleUnit.RADIANS);
        headingOffset = pose.getHeading() - rawHeading;

        double cos = Math.cos(headingOffset);
        double sin = Math.sin(headingOffset);
        double rotatedRawX = (raw.getX(DistanceUnit.INCH) * cos) - (raw.getY(DistanceUnit.INCH) * sin);
        double rotatedRawY = (raw.getX(DistanceUnit.INCH) * sin) + (raw.getY(DistanceUnit.INCH) * cos);
        translationOffsetX = pose.getX() - rotatedRawX;
        translationOffsetY = pose.getY() - rotatedRawY;

        poseEstimate = pose;
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
        double rawHeading = pos.getHeading(AngleUnit.RADIANS);
        double cos = Math.cos(headingOffset);
        double sin = Math.sin(headingOffset);
        double rotatedX = (pos.getX(DistanceUnit.INCH) * cos) - (pos.getY(DistanceUnit.INCH) * sin);
        double rotatedY = (pos.getX(DistanceUnit.INCH) * sin) + (pos.getY(DistanceUnit.INCH) * cos);
        poseEstimate = new Pose2d(
                translationOffsetX + rotatedX,
                translationOffsetY + rotatedY,
                rawHeading + headingOffset
        );
    }
}
