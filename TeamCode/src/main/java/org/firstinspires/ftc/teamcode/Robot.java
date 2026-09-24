package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.control.DriveProfile;
import org.firstinspires.ftc.teamcode.control.DriveSignal;
import org.firstinspires.ftc.teamcode.control.MecanumMixer;
import org.firstinspires.ftc.teamcode.control.MoveToResult;
import org.firstinspires.ftc.teamcode.control.RobotActions;
import org.firstinspires.ftc.teamcode.control.WheelPowers;
import org.firstinspires.ftc.teamcode.subsystem.Collector;
import org.firstinspires.ftc.teamcode.subsystem.CollectorMode;
import org.firstinspires.ftc.teamcode.subsystem.Drive;
import org.firstinspires.ftc.teamcode.util.PIDController;

/**
 * Mode-independent owner of robot hardware and core behaviors.
 *
 * Gamepads, autonomous plans, FTC OpModes, Dashboard, and telemetry deliberately live outside this
 * class. Teleop and autonomous therefore request the same drive and collector behaviors.
 */
public class Robot implements RobotActions {
    private static final double MAX_CONTROLLER_DT_SECONDS = 0.1;

    // These time-based defaults approximate the prior per-loop gains at a nominal 50 Hz loop.
    public static final double DEFAULT_TRANSLATION_KP_POWER_PER_MM = 0.006;
    public static final double DEFAULT_TRANSLATION_KI_POWER_PER_MM_SECOND = 0.03;
    public static final double DEFAULT_TRANSLATION_KD_POWER_SECOND_PER_MM = 0.0004;
    public static final double DEFAULT_ROTATION_KP_POWER_PER_RADIAN = 2 * Math.PI / 3;
    public static final double DEFAULT_ROTATION_KI_POWER_PER_RADIAN_SECOND = 0.03;
    public static final double DEFAULT_ROTATION_KD_POWER_SECOND_PER_RADIAN = 0.0004;

    private final GoBildaPinpointDriver pinpoint;
    private final Drive drive;
    private final Collector collector;
    private final PIDController forwardController = new PIDController(
            DEFAULT_TRANSLATION_KP_POWER_PER_MM,
            DEFAULT_TRANSLATION_KI_POWER_PER_MM_SECOND,
            DEFAULT_TRANSLATION_KD_POWER_SECOND_PER_MM);
    private final PIDController leftController = new PIDController(
            DEFAULT_TRANSLATION_KP_POWER_PER_MM,
            DEFAULT_TRANSLATION_KI_POWER_PER_MM_SECOND,
            DEFAULT_TRANSLATION_KD_POWER_SECOND_PER_MM);
    private final PIDController headingController = new PIDController(
            DEFAULT_ROTATION_KP_POWER_PER_RADIAN,
            DEFAULT_ROTATION_KI_POWER_PER_RADIAN_SECOND,
            DEFAULT_ROTATION_KD_POWER_SECOND_PER_RADIAN);

    public Robot(HardwareMap hardwareMap) {
        drive = new Drive(hardwareMap);
        collector = new Collector(hardwareMap);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setOffsets(-100.0, -30.0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
        setPose(new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0));
    }

    /** Refreshes sensors only. It does not change actuator demands. */
    public void updateSensors() {
        pinpoint.update();
    }

    public Pose2D getPose() {
        return pinpoint.getPosition();
    }

    public void setPose(Pose2D pose) {
        pinpoint.setPosition(pose);
        resetMoveToControllers();
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

    /** Applies a robot-relative drive signal through the one shared mecanum mixer. */
    public WheelPowers driveRobotRelative(DriveSignal requestedSignal) {
        WheelPowers wheelPowers = MecanumMixer.mix(requestedSignal);
        drive.setDrivePowers(
                wheelPowers.getFrontLeft(),
                wheelPowers.getFrontRight(),
                wheelPowers.getBackLeft(),
                wheelPowers.getBackRight());
        return wheelPowers;
    }

    /** Performs one nonblocking update toward an absolute field pose. */
    public MoveToResult moveTo(Pose2D targetPose, DriveProfile profile, double dtSeconds) {
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

        double robotForwardError = fieldXError * Math.cos(currentHeading)
                + fieldYError * Math.sin(currentHeading);
        double robotLeftError = -fieldXError * Math.sin(currentHeading)
                + fieldYError * Math.cos(currentHeading);

        boolean atTarget = distanceError <= profile.getPositionToleranceMm()
                && Math.abs(headingError) <= profile.getHeadingToleranceRadians();
        if (atTarget) {
            stopDrive();
            resetMoveToControllers();
            return new MoveToResult(
                    currentPose, targetPose,
                    fieldXError, fieldYError,
                    robotForwardError, robotLeftError,
                    headingError,
                    DriveSignal.ZERO, WheelPowers.ZERO, true);
        }

        double controllerDt = Range.clip(dtSeconds, 1e-4, MAX_CONTROLLER_DT_SECONDS);
        double forwardPower = forwardController.calculate(robotForwardError, 0.0, controllerDt);
        double leftPower = leftController.calculate(robotLeftError, 0.0, controllerDt);
        double counterclockwisePower = headingController.calculate(headingError, 0.0, controllerDt);

        double translationPower = Math.hypot(forwardPower, leftPower);
        if (translationPower > profile.getMaxTranslationPower() && translationPower > 0.0) {
            double translationScale = profile.getMaxTranslationPower() / translationPower;
            forwardPower *= translationScale;
            leftPower *= translationScale;
        }
        counterclockwisePower = Range.clip(
                counterclockwisePower,
                -profile.getMaxRotationPower(),
                profile.getMaxRotationPower());

        DriveSignal driveSignal = new DriveSignal(
                forwardPower, leftPower, counterclockwisePower);
        WheelPowers wheelPowers = driveRobotRelative(driveSignal);
        return new MoveToResult(
                currentPose, targetPose,
                fieldXError, fieldYError,
                robotForwardError, robotLeftError,
                headingError,
                driveSignal, wheelPowers, false);
    }

    public void setTranslationPid(double kP, double kI, double kD) {
        forwardController.kP = kP;
        forwardController.kI = kI;
        forwardController.kD = kD;
        leftController.kP = kP;
        leftController.kI = kI;
        leftController.kD = kD;
    }

    public void setRotationPid(double kP, double kI, double kD) {
        headingController.kP = kP;
        headingController.kI = kI;
        headingController.kD = kD;
    }

    public void resetMoveToControllers() {
        forwardController.reset();
        leftController.reset();
        headingController.reset();
    }

    public void setCollectorPower(double power) {
        collector.setPower(Range.clip(power, -1.0, 1.0));
    }

    public void setCollectorMode(CollectorMode mode) {
        setCollectorPower(mode.getPower());
    }

    public void stopDrive() {
        drive.setDrivePowers(0.0, 0.0, 0.0, 0.0);
    }

    public void stop() {
        stopDrive();
        setCollectorMode(CollectorMode.OFF);
        resetMoveToControllers();
    }
}
