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

    public static Pose2d approachPose(Pose2d targetPose, double offsetDistance) {
        // Calculate offset components based on target heading
        double offsetX = offsetDistance * Math.cos(targetPose.getHeading());
        double offsetY = offsetDistance * Math.sin(targetPose.getHeading());

        // Calculate the robot's position by subtracting the offset from the target position
        double robotX = targetPose.getX() - offsetX;
        double robotY = targetPose.getY() - offsetY;

        // Return the robot's required pose
        return new Pose2d(robotX, robotY, targetPose.getHeading());
    }

    public static Pose2d collectApproachPose(Pose2d targetPose) {
        return approachPose(targetPose, NibusConstants.COLLECT_HEAD_BASE_OFFSET_X);
    }
}
