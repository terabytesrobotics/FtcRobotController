package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.Gamepad;

public class NibusHelpers {

    public static Vector2d headlessLeftStickFieldDirection(Gamepad gamepad, double operatorAngleOffset, double robotHeading) {
        // Get the gamepad stick inputs and normalize them
        double rawGamepadY = gamepad.left_stick_y;
        double rawGamepadX = gamepad.left_stick_x;
        double normalizedGamepadY = -rawGamepadY; // Inverting Y to normalize direction
        double normalizedGamepadX = rawGamepadX;
        double inputMagnitude = Math.hypot(normalizedGamepadX, normalizedGamepadY);
        double operatorRelativeStickHeading = Angle.norm(Math.atan2(normalizedGamepadY, normalizedGamepadX) - (Math.PI / 2));
        double operatorFieldStickHeading = Angle.norm(operatorRelativeStickHeading + operatorAngleOffset);
        double robotRelativeTranslationHeading = Angle.norm(operatorFieldStickHeading - robotHeading);
        double robotX = inputMagnitude * Math.cos(robotRelativeTranslationHeading);
        double robotY = inputMagnitude * Math.sin(robotRelativeTranslationHeading);
        return new Vector2d(robotX, robotY);
    }

    public static Pose2d robotPose(Pose2d appendagePose, double inlineOffset, double orthogonalOffset) {
        double appendageHeading = appendagePose.getHeading();
        double headingOffset = Math.atan2(inlineOffset, orthogonalOffset);
        double distance = Math.hypot(inlineOffset, orthogonalOffset);
        double robotHeading = Angle.norm(appendageHeading - headingOffset);
        double robotX = appendagePose.getX() - (distance * Math.cos(robotHeading));
        double robotY = appendagePose.getY() - (distance * Math.sin(robotHeading));
        return new Pose2d(robotX, robotY, robotHeading);
    }

    public static Pose2d appendagePose(Pose2d robotPose, double inlineOffset, double orthogonalOffset) {
        double robotHeading = robotPose.getHeading();
        double headingOffset = Math.atan2(inlineOffset, orthogonalOffset);
        double appendageHeading = Angle.norm(robotHeading + headingOffset);
        double distance = Math.hypot(inlineOffset, orthogonalOffset);
        double appendageX = robotPose.getX() + (distance * Math.cos(appendageHeading));
        double appendageY = robotPose.getY() + (distance * Math.sin(appendageHeading));
        return new Pose2d(appendageX, appendageY, appendageHeading);
    }
}
