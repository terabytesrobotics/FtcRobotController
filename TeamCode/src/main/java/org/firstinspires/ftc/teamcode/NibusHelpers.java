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
        return robotPose(appendagePose, inlineOffset, orthogonalOffset, 0);
    }

    public static Pose2d robotPose(Pose2d appendagePose, double inlineOffset, double orthogonalOffset, double appendageRobotHeadingOffset) {

        double appendageHeadingOffset = Math.atan2(orthogonalOffset, inlineOffset);
        double effectiveAppendageHeading = Angle.normDelta(appendagePose.getHeading() + appendageHeadingOffset);
        double distance = Math.hypot(inlineOffset, orthogonalOffset);
        double robotHeading = Angle.normDelta(effectiveAppendageHeading + appendageRobotHeadingOffset);
        double robotX = appendagePose.getX() - (distance * Math.cos(robotHeading));
        double robotY = appendagePose.getY() - (distance * Math.sin(robotHeading));
        return new Pose2d(robotX, robotY, robotHeading);
    }

    public static Pose2d robotPose2(Pose2d desiredAppendagePose, double appendageXLength, double appendageYLength, double appendageRobotHeadingOffset) {
        double appendageHeading = desiredAppendagePose.getHeading();
        double appendageLength = Math.hypot(appendageXLength, appendageYLength);
        double appendageEffectiveHeadingOffset = Math.atan2(appendageYLength, appendageXLength);
        double effectiveAppendageHeading = appendageHeading + appendageEffectiveHeadingOffset;
        double robotX = desiredAppendagePose.getX() - (appendageLength * Math.cos(effectiveAppendageHeading));
        double robotY = desiredAppendagePose.getY() - (appendageLength * Math.sin(effectiveAppendageHeading));
        double robotHeading = appendageHeading + appendageRobotHeadingOffset;
        return new Pose2d(robotX, robotY, robotHeading);
    }

    public static Pose2d appendagePose(Pose2d robotPose, double inlineOffset, double orthogonalOffset) {
        double robotHeading = robotPose.getHeading();
        double headingOffset = Math.atan2(orthogonalOffset, inlineOffset);
        double appendageHeading = Angle.norm(robotHeading + headingOffset);
        double distance = Math.hypot(inlineOffset, orthogonalOffset);
        double appendageX = robotPose.getX() + (distance * Math.cos(appendageHeading));
        double appendageY = robotPose.getY() + (distance * Math.sin(appendageHeading));
        return new Pose2d(appendageX, appendageY, appendageHeading);
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
        double heightAtMaxExtension = ((-b) + Math.sqrt((b*b) - (4 * a * c))) / (2 * a);
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

    private Pose2d getRobotHeadedFinesseInput(boolean isUpstage, Gamepad gamepad, Pose2d fixedPose) {
        final double FORWARD_ALLOWANCE = 18;
        final double LATERAL_ALLOWANCE = 6;
        final double THETA_ALLOWANCE = Math.PI / 4;

        double forwardHeading = isUpstage ?
                Angle.norm(fixedPose.getHeading() + Math.PI) :
                fixedPose.getHeading();
        double desiredForwardOffset = -gamepad.left_stick_y * FORWARD_ALLOWANCE;
        double desiredLateralOffset = gamepad.left_stick_x * LATERAL_ALLOWANCE;
        double desiredAngleOffset = -gamepad.right_stick_x * THETA_ALLOWANCE;

        double withForwardX = fixedPose.getX() + (desiredForwardOffset * Math.cos(forwardHeading));
        double withForwardY = fixedPose.getY() + (desiredForwardOffset * Math.sin(forwardHeading));

        double orthogonalDirection = Angle.norm(forwardHeading + (Math.PI / 2));
        double desiredX = withForwardX + (desiredLateralOffset * Math.cos(orthogonalDirection));
        double desiredY = withForwardY + (desiredLateralOffset * Math.sin(orthogonalDirection));

        double desiredHeading = Angle.norm(fixedPose.getHeading() + desiredAngleOffset);

        return new Pose2d(desiredX, desiredY, desiredHeading);
    }

    public static Pose2d getPreScoringPose(AllianceColor allianceColor, CenterStageBackdropPosition backdropPosition, double xOffset, double yOffset) {
        Pose2d scoringFocalPoint =
                allianceColor
                        .getAprilTagForScoringPosition(backdropPosition)
                        .facingPose();
        return NibusHelpers.robotPose2(scoringFocalPoint, xOffset, yOffset, 0);
    }
}
