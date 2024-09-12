package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.NibusConstants.ARM_TICKS_PER_DEGREE;
import static org.firstinspires.ftc.teamcode.NibusConstants.EXTENDER_MAX_LENGTH_INCHES;
import static org.firstinspires.ftc.teamcode.NibusConstants.EXTENDER_TICS_PER_INCH;
import static org.firstinspires.ftc.teamcode.NibusConstants.SCORING_HEIGHT_MAX;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.opencv.core.Mat;

public class TerabytesHelpers {

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

    public static Mat.Tuple4<Double> armExtenderWristAndOffsetForScoringHeight(double scoringHeightInches) {

        double ARM_MINIMUM_LENGTH_INCHES = 16.5;
        double ROBOT_X_OFFSET_LOW_SCORING = 21;
        double ARM_MAX_TOTAL_LENGTH = ARM_MINIMUM_LENGTH_INCHES + EXTENDER_MAX_LENGTH_INCHES;
        double BACKDROP_ANGLE = Math.PI / 3;
        int HORIZONTAL_TICKS = 5818;
        double WRIST_LOW_SETPOINT = .67;
        double WRIST_HIGH_SETPOINT = .39;
        double WRIST_RANGE = WRIST_HIGH_SETPOINT - WRIST_LOW_SETPOINT;
        double percentageOfTotalHeight = scoringHeightInches / SCORING_HEIGHT_MAX;
        double wristPosition = WRIST_LOW_SETPOINT + (percentageOfTotalHeight * WRIST_RANGE);

        double a = (1 + (1 / Math.tan(BACKDROP_ANGLE)));
        double b = (2 * ARM_MINIMUM_LENGTH_INCHES) / Math.tan(BACKDROP_ANGLE);
        double c = ((ARM_MINIMUM_LENGTH_INCHES * ARM_MINIMUM_LENGTH_INCHES) - (ARM_MAX_TOTAL_LENGTH * ARM_MAX_TOTAL_LENGTH));
        double heightAtMaxExtension = ((-b) + Math.sqrt((b * b) - (4 * a * c))) / (2 * a);
        double xAtMaxExtension = ARM_MINIMUM_LENGTH_INCHES + (heightAtMaxExtension / Math.tan(BACKDROP_ANGLE));

        if (scoringHeightInches < heightAtMaxExtension) {
            double xBackdrop = scoringHeightInches / Math.tan(BACKDROP_ANGLE);
            double x = ARM_MINIMUM_LENGTH_INCHES + xBackdrop;
            double y = scoringHeightInches;
            double armAngle = Math.atan2(y, x);
            double length = Math.sqrt((x * x) + (y * y));
            double extension = Math.min(length - ARM_MINIMUM_LENGTH_INCHES, EXTENDER_MAX_LENGTH_INCHES);
            double armTargetTicks = HORIZONTAL_TICKS - (Math.toDegrees(armAngle) * ARM_TICKS_PER_DEGREE);
            double extenderTargetTicks = extension * EXTENDER_TICS_PER_INCH;

            return new Mat.Tuple4<>(armTargetTicks, extenderTargetTicks, wristPosition, ROBOT_X_OFFSET_LOW_SCORING);
        } else {
            double heightOverMaxLowHeight = scoringHeightInches - heightAtMaxExtension;
            double removedBackdropX = heightOverMaxLowHeight / Math.tan(BACKDROP_ANGLE);
            double extenderTargetTicks = EXTENDER_MAX_LENGTH_INCHES * EXTENDER_TICS_PER_INCH;
            double x = xAtMaxExtension - removedBackdropX;
            double y = scoringHeightInches;
            double armAngle = Math.atan2(y, x);
            double armTargetTicks = HORIZONTAL_TICKS - (Math.toDegrees(armAngle) * ARM_TICKS_PER_DEGREE);
            return new Mat.Tuple4<>(armTargetTicks, extenderTargetTicks, wristPosition, ROBOT_X_OFFSET_LOW_SCORING - removedBackdropX);
        }
    }

    public static Pose2d getPreScoringPose(AllianceColor allianceColor, CenterStageBackdropPosition backdropPosition, double xOffset, double yOffset) {
        Pose2d scoringFocalPoint =
                allianceColor
                        .getAprilTagForScoringPosition(backdropPosition)
                        .facingPose();
        return TerabytesHelpers.robotPoseForDesiredAppendagePose(scoringFocalPoint, xOffset, yOffset, 0);
    }
}
