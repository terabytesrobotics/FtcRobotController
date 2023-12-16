package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.pointsOfInterest;

public enum NibusAutonomousPlan {
    PARK_LEFT(pointsOfInterest.redLeftBackstagePark, pointsOfInterest.blueLeftBackstagePark),
    PARK_RIGHT(pointsOfInterest.redRightBackstagePark, pointsOfInterest.blueRightBackstagePark);

    public final pointsOfInterest RedParkLocation;
    public final pointsOfInterest BlueParkLocation;

    NibusAutonomousPlan(pointsOfInterest redParkLocation, pointsOfInterest blueParkLocation) {
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
