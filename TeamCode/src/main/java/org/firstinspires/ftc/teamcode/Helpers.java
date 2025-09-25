package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Helpers {

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

    public static Vector2d headlessABButtonFieldDirection(Gamepad gamepad, double operatorAngleOffset, double robotHeading) {
        double magnitude = 1.0;
        double operatorRelativeHeading;
        if (gamepad.y && !gamepad.a) {
            operatorRelativeHeading = 0;
        } else if (gamepad.a && !gamepad.y) {
            operatorRelativeHeading = Math.PI;
        } else {
            return new Vector2d(0, 0);
        }
        double operatorFieldHeading = Angle.norm(operatorRelativeHeading + operatorAngleOffset);
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
