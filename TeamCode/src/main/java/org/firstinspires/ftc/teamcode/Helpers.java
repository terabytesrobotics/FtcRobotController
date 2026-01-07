package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Helpers {

    /**
     * Maps the left stick (up = +Y operator, right = +X) into robot-frame X/Y.
     * driverForwardHeading is the field heading the driver faces; right is driverForwardHeading - PI/2.
     */
    public static Vector2d fieldRelativeLeftStick(Gamepad gamepad, double driverForwardHeading, double robotHeading) {
        double operatorX = gamepad.left_stick_x;
        double operatorY = -gamepad.left_stick_y; // invert so up is +Y
        if (Math.abs(operatorX) < 1e-4 && Math.abs(operatorY) < 1e-4) return new Vector2d(0, 0);

        double rightHeading = driverForwardHeading - (Math.PI / 2.0);
        double fieldX = (operatorX * Math.cos(rightHeading)) + (operatorY * Math.cos(driverForwardHeading));
        double fieldY = (operatorX * Math.sin(rightHeading)) + (operatorY * Math.sin(driverForwardHeading));

        // Rotate field vector into robot frame
        double cosH = Math.cos(robotHeading);
        double sinH = Math.sin(robotHeading);
        double robotX = (fieldX * cosH) + (fieldY * sinH);
        double robotY = (-fieldX * sinH) + (fieldY * cosH);

        double mag = Math.hypot(robotX, robotY);
        if (mag > 1.0) {
            robotX /= mag;
            robotY /= mag;
        }
        return new Vector2d(robotX, robotY);
    }

    public static Vector2d headlessABButtonFieldDirection(Gamepad gamepad, double driverForwardHeading, double robotHeading) {
        double magnitude = 1.0;
        double operatorRelativeHeading;
        if (gamepad.y && !gamepad.a) {
            operatorRelativeHeading = 0;
        } else if (gamepad.a && !gamepad.y) {
            operatorRelativeHeading = Math.PI;
        } else {
            return new Vector2d(0, 0);
        }
        double operatorFieldHeading = Angle.norm(operatorRelativeHeading + driverForwardHeading);
        double robotRelativeHeading = Angle.norm(operatorFieldHeading - robotHeading);
        return new Vector2d(magnitude * Math.cos(robotRelativeHeading), magnitude * Math.sin(robotRelativeHeading));
    }

    public static Pose2d robotPoseForDesiredAppendagePose(Pose2d desiredAppendagePose, double appendageXLength, double appendageYLength, double appendageRobotHeadingOffset) {
        double appendageHeading = desiredAppendagePose.getHeading();
        double appendageLength = Math.hypot(appendageXLength, appendageYLength);
        double appendageEffectiveHeadingOffset = Math.atan2(appendageYLength, appendageXLength);
        double effectiveAppendageHeading = appendageHeading + appendageEffectiveHeadingOffset;
        double robotX = desiredAppendagePose.getX() - (appendageLength * Math.cos(effectiveAppendageHeading));
        double robotY = desiredAppendagePose.getY() - (appendageLength * Math.sin(effectiveAppendageHeading));
        double robotHeading = appendageHeading + appendageRobotHeadingOffset;
        return new Pose2d(robotX, robotY, robotHeading);
    }
}
