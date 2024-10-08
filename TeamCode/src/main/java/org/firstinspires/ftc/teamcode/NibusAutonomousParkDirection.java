package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.PointOfInterest;

public enum NibusAutonomousParkDirection {
    PARK_INSIDE(PointOfInterest.redLeftBackstagePark, PointOfInterest.blueRightBackstagePark),
    PARK_OUTSIDE(PointOfInterest.redRightBackstagePark, PointOfInterest.blueLeftBackstagePark);

    public final PointOfInterest RedParkLocation;
    public final PointOfInterest BlueParkLocation;

    NibusAutonomousParkDirection(PointOfInterest redParkLocation, PointOfInterest blueParkLocation) {
        RedParkLocation = redParkLocation;
        BlueParkLocation = blueParkLocation;
    }

    public Vector2d getParkLocation(AllianceColor allianceColor) {
        switch (allianceColor) {
            case RED:
                return new Vector2d(RedParkLocation.X, RedParkLocation.Y);
            case BLUE:
                return new Vector2d(BlueParkLocation.X, BlueParkLocation.Y);
            default:
                return new Vector2d();
        }
    }
}
