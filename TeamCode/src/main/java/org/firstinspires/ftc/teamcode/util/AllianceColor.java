package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

public enum AllianceColor {
    // Heading the driver faces (their forward/+Y stick direction) in field coordinates (CCW-positive).
    BLUE(Math.toRadians(-90)), // drivers on +Y wall facing -Y
    RED(Math.toRadians(90));   // drivers on -Y wall facing +Y

    public final double driverForwardHeading;

    AllianceColor(double driverForwardHeading) {
        this.driverForwardHeading = driverForwardHeading;
    }

    public double intoTheDeepNetApproachHeadingX() {
        return this == RED ? Math.toRadians(180) : Math.toRadians(0);
    }

    public double intoTheDeepNetApproachHeadingY() {
        return this == RED ? Math.toRadians(270) : Math.toRadians(90);
    }

    public Vector2d getMiddleLaneAudienceWaypoint() {
        switch (this) {
            case RED:
                return new Vector2d(PointOfInterest.redGateAudience.X, PointOfInterest.redGateAudience.Y);
            case BLUE:
                return new Vector2d(PointOfInterest.blueGateAudience.X, PointOfInterest.blueGateAudience.Y);
            default:
                return new Vector2d();
        }
    }

    public Vector2d getMiddleLaneBackstageWaypoint() {
        switch (this) {
            case RED:
                return new Vector2d(PointOfInterest.redGateBack.X, PointOfInterest.redGateBack.Y);
            case BLUE:
                return new Vector2d(PointOfInterest.blueGateBack.X, PointOfInterest.blueGateBack.Y);
            default:
                return new Vector2d();
        }
    }

    public Vector2d getInnerLaneBackstageWaypoint() {
        switch (this) {
            case RED:
                return new Vector2d(PointOfInterest.redInnerBack.X, PointOfInterest.redInnerBack.Y);
            case BLUE:
                return new Vector2d(PointOfInterest.blueInnerBack.X, PointOfInterest.blueInnerBack.Y);
            default:
                return new Vector2d();
        }
    }

    public Vector2d getScoringPreApproachLocation() {
        switch (this) {
            case RED:
                return new Vector2d(PointOfInterest.redScoringPreApproach.X, PointOfInterest.redScoringPreApproach.Y);
            case BLUE:
                return new Vector2d(PointOfInterest.blueScoringPreApprach.X, PointOfInterest.blueScoringPreApprach.Y);
            default:
                return new Vector2d();
        }
    }

    public Vector2d getScoringApproachLocation() {
        switch (this) {
            case RED:
                return new Vector2d(PointOfInterest.redScoringApproach.X, PointOfInterest.redScoringApproach.Y);
            case BLUE:
                return new Vector2d(PointOfInterest.blueScoringApprach.X, PointOfInterest.blueScoringApprach.Y);
            default:
                return new Vector2d();
        }
    }

    public RevBlinkinLedDriver.BlinkinPattern getAllianceColorBlinkinPattern() {
        switch (this) {
            case RED:
                return RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED;
            case BLUE:
            default:
                return RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE;
        }
    }

    public Pose2d getTeleopScoringApproachLocation() {
        switch (this) {
            case RED:
                return new Pose2d(PointOfInterest.RED_BACKDROP_APPROACH.X, PointOfInterest.RED_BACKDROP_APPROACH.Y, 0);
            case BLUE:
            default:
                return new Pose2d(PointOfInterest.BLUE_BACKDROP_APPROACH.X, PointOfInterest.BLUE_BACKDROP_APPROACH.Y, 0);
        }
    }
}
