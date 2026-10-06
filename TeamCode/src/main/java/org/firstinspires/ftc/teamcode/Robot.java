package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.drive.DriveSignal;
import org.firstinspires.ftc.teamcode.drive.MecanumMixer;
import org.firstinspires.ftc.teamcode.drive.WheelPowers;
import org.firstinspires.ftc.teamcode.subsystem.Collector;
import org.firstinspires.ftc.teamcode.subsystem.Drive;
import org.firstinspires.ftc.teamcode.util.MathHelper;
import org.firstinspires.ftc.teamcode.util.PIDController;

// https://github.com/FIRST-Tech-Challenge/FtcRobotController/blob/master/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/SensorLimelight3A.java
public class Robot {
    GoBildaPinpointDriver pinpoint;
    public Drive drive;
    public Collector collector;
    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    private final int moveToThreshold = 5;
    private final double rotateToThreshold = Math.PI / 32.0;
    private final double minPunchApplication = 0.05;
    private final double maxPunchApplication = 0.3;
    private final double punch = 0.5;
    public final double kP_POWER_PER_MM = 0.006;
    public final double kI_POWER_PER_MM_SEC = 0.015;
    public final double kD_POWER_PER_MM_PER_SEC = 0.001;
    public final PIDController xDriveController = new PIDController(kP_POWER_PER_MM, kI_POWER_PER_MM_SEC, kD_POWER_PER_MM_PER_SEC);
    public final PIDController yDriveController = new PIDController(kP_POWER_PER_MM, kI_POWER_PER_MM_SEC, kD_POWER_PER_MM_PER_SEC);
    public final PIDController rDriveController = new PIDController(2 * Math.PI / 3, kI_POWER_PER_MM_SEC, kD_POWER_PER_MM_PER_SEC);
    Limelight3A limelight;
    private double maxTranslationPower = 0.9;
    private double maxRotationPower = 0.60;

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

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        this.hardwareMap = hardwareMap;
//        this.telemetry = telemetry;
        this.telemetry = new MultipleTelemetry(
                telemetry, FtcDashboard.getInstance().getTelemetry());
        this.telemetry.setMsTransmissionInterval(50);
    }

    public void update() {
        drive.setDrivePowers(0, 0, 0, 0);
        pinpoint.update();
        telemetry.update();
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

    public boolean moveToRot(double targetX, double targetY, double targetRad, double dtSeconds) {
        return moveToRot(targetX, targetY, targetRad, dtSeconds, false);
    }

    public boolean moveToRot(
            double targetX,
            double targetY,
            double targetRad,
            double dtSeconds,
            boolean debug
    ) {
        Pose2D currentPose = pinpoint.getPosition();

        double currentFieldX = currentPose.getX(DistanceUnit.MM);
        double currentFieldY = currentPose.getY(DistanceUnit.MM);
        double currentHeading = currentPose.getHeading(AngleUnit.RADIANS);

        double fieldXError = targetX - currentFieldX;
        double fieldYError = targetY - currentFieldY;
        double distanceError = Math.hypot(fieldXError, fieldYError);
        double headingError = AngleUnit.normalizeRadians(targetRad - currentHeading);

        double robotForwardError = fieldXError * Math.cos(currentHeading)
                + fieldYError * Math.sin(currentHeading);
        double robotLeftError = -fieldXError * Math.sin(currentHeading)
                + fieldYError * Math.cos(currentHeading);

        boolean atTarget = distanceError <= moveToThreshold
                && Math.abs(headingError) <= rotateToThreshold;
        if (atTarget) {
            drive.stop();

            resetControllers();
            return true;
        }

        double controllerDt = Range.clip(dtSeconds, 1e-4, .1);
        double forwardPower = yDriveController.calculate(robotForwardError, 0.0, controllerDt);
        double leftPower = xDriveController.calculate(robotLeftError, 0.0, controllerDt);
        double hyp = Math.hypot(forwardPower, leftPower);
        double counterclockwisePower = rDriveController.calculate(headingError, 0.0, controllerDt);

        if (hyp > maxTranslationPower && hyp > 0.0) {
            double translationScale = maxTranslationPower / hyp;
            forwardPower *= translationScale;
            leftPower *= translationScale;
        }
        // apply punch
        if (hyp > minPunchApplication && hyp < maxPunchApplication) {
            double scale = punch / hyp;
            forwardPower *= scale;
            leftPower *= scale;
        }

        counterclockwisePower = Range.clip(
                counterclockwisePower,
                -maxRotationPower,
                maxRotationPower);

        DriveSignal driveSignal = new DriveSignal(
                forwardPower, leftPower, counterclockwisePower);
        driveRobotRelative(driveSignal);
        return false;
    }

    public WheelPowers driveRobotRelative(DriveSignal requestedSignal) {
        WheelPowers wheelPowers = MecanumMixer.mix(requestedSignal);
        drive.setDrivePowers(
                wheelPowers.getFrontLeft(),
                wheelPowers.getFrontRight(),
                wheelPowers.getBackLeft(),
                wheelPowers.getBackRight());
        return wheelPowers;
    }

    public void resetControllers() {
        xDriveController.reset();
        yDriveController.reset();
        rDriveController.reset();
    }
}
