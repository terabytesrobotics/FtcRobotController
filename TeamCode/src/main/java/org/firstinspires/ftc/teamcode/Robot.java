package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystem.Collector;
import org.firstinspires.ftc.teamcode.subsystem.Drive;
import org.firstinspires.ftc.teamcode.util.PIDController;

public class Robot {
    GoBildaPinpointDriver pinpoint;
    public Drive drive;
    public Collector collector;
    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    private final int moveToThreshold = 5;
    private final double rotateToThreshold = Math.PI / 16.0; // radians
    private final double MAX_DRIVE_OUTPUT_POWER = 0.95;
    public final double kP_POWER_PER_MM = .006; // 100% power (~torque) / 1000mm
    public final double kI_POWER_PER_MM_SEC = 0.0006; // 0% power / mm * sec
    public final double kD_POWER_PER_MM_PER_SEC = 0.02; // 0% power / (mm/sec)
    public final PIDController xDriveController = new PIDController(kP_POWER_PER_MM, kI_POWER_PER_MM_SEC, kD_POWER_PER_MM_PER_SEC);
    public final PIDController yDriveController = new PIDController(kP_POWER_PER_MM, kI_POWER_PER_MM_SEC, kD_POWER_PER_MM_PER_SEC);
    public final PIDController rDriveController = new PIDController(2 * Math.PI / 3, kI_POWER_PER_MM_SEC, kD_POWER_PER_MM_PER_SEC);

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        drive = new Drive(hardwareMap);
        collector = new Collector(hardwareMap);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // configure pinpoint
        pinpoint.setOffsets(-100.0, -30.0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();

        // set the starting location
        pinpoint.setPosition(new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0));

        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    public void update() {
        drive.setDrivePowers(0, 0, 0, 0);
        pinpoint.update();
    }

    public Pose2D getPos() {
        return pinpoint.getPosition();
    }

    public double getX() {
        return pinpoint.getPosX(DistanceUnit.MM);
    }

    public double getY() {
        return pinpoint.getPosY(DistanceUnit.MM);
    }

    public double getHeadingDeg() {
        return pinpoint.getHeading(AngleUnit.DEGREES);
    }

    public double getHeadingRad() {
        return pinpoint.getHeading(AngleUnit.RADIANS);
    }

    /**
     * Drives toward one absolute field pose.
     *
     * Pinpoint coordinates use +X forward, +Y left, and positive heading counterclockwise
     * when the field pose is initialized at heading zero.
     *
     * @return true when both translation and heading are within their thresholds
     */
    public boolean moveTo(Pose2D targetPose, boolean debug) {
        Pose2D currentPose = pinpoint.getPosition();

        double targetFieldX = targetPose.getX(DistanceUnit.MM);
        double targetFieldY = targetPose.getY(DistanceUnit.MM);
        double targetHeading = targetPose.getHeading(AngleUnit.RADIANS);

        double currentFieldX = currentPose.getX(DistanceUnit.MM);
        double currentFieldY = currentPose.getY(DistanceUnit.MM);
        double currentHeading = currentPose.getHeading(AngleUnit.RADIANS);

        double fieldXError = targetFieldX - currentFieldX;
        double fieldYError = targetFieldY - currentFieldY;
        double distanceError = Math.hypot(fieldXError, fieldYError);
        double headingError = AngleUnit.normalizeRadians(targetHeading - currentHeading);

        if (distanceError < moveToThreshold && Math.abs(headingError) < rotateToThreshold) {
            drive.setDrivePowers(0, 0, 0, 0);
            return true;
        }

        // Rotate the field-relative translation error into the robot's coordinate frame.
        double robotForwardError = fieldXError * Math.cos(currentHeading)
                + fieldYError * Math.sin(currentHeading);
        double robotLeftError = -fieldXError * Math.sin(currentHeading)
                + fieldYError * Math.cos(currentHeading);

        // The errors are already calculated and, for heading, wrapped to the shortest turn.
        double forwardPower = xDriveController.calculate(robotForwardError, 0);
        double leftPower = yDriveController.calculate(robotLeftError, 0);
        double counterclockwisePower = rDriveController.calculate(headingError, 0);

        // Mix robot-relative forward, left, and counterclockwise commands into wheel powers.
        double frontLeftPower = forwardPower - leftPower - counterclockwisePower;
        double frontRightPower = forwardPower + leftPower + counterclockwisePower;
        double backLeftPower = forwardPower + leftPower - counterclockwisePower;
        double backRightPower = forwardPower - leftPower + counterclockwisePower;

        double maximumRequestedPower = Math.max(
                Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
        double powerScale = maximumRequestedPower > MAX_DRIVE_OUTPUT_POWER
                ? MAX_DRIVE_OUTPUT_POWER / maximumRequestedPower
                : 1.0;

        if (debug) {
            telemetry.addData("Current field X/Y", "%.1f / %.1f", currentFieldX, currentFieldY);
            telemetry.addData("Target field X/Y", "%.1f / %.1f", targetFieldX, targetFieldY);
            telemetry.addData("Field error X/Y", "%.1f / %.1f", fieldXError, fieldYError);
            telemetry.addData("Robot error forward/left", "%.1f / %.1f", robotForwardError, robotLeftError);
            telemetry.addData("Distance error", "%.1f mm", distanceError);
            telemetry.addData("Heading current/target/error", "%.3f / %.3f / %.3f",
                    currentHeading, targetHeading, headingError);
            telemetry.addData("Command forward/left/CCW", "%.3f / %.3f / %.3f",
                    forwardPower, leftPower, counterclockwisePower);
            telemetry.addData("Raw wheels FL/FR/BL/BR", "%.3f / %.3f / %.3f / %.3f",
                    frontLeftPower, frontRightPower, backLeftPower, backRightPower);
            telemetry.addData("Power scale", "%.3f", powerScale);
        }

        drive.setDrivePowers(
                frontLeftPower * powerScale,
                frontRightPower * powerScale,
                backLeftPower * powerScale,
                backRightPower * powerScale);

        return false;
    }
}
