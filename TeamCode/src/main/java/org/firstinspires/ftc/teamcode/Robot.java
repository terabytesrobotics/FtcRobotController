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
    private final double rotateToThreshold = Math.PI / 16.0; // radians
    private final double MAX_DRIVE_OUTPUT_POWER = 0.95;
    public final double kP_POWER_PER_MM = .006; // 100% power (~torque) / 1000mm
    public final double kI_POWER_PER_MM_SEC = 0.0006; // 0% power / mm * sec
    public final double kD_POWER_PER_MM_PER_SEC = 0.02; // 0% power / (mm/sec)
    public final PIDController xDriveController = new PIDController(kP_POWER_PER_MM, kI_POWER_PER_MM_SEC, kD_POWER_PER_MM_PER_SEC);
    public final PIDController yDriveController = new PIDController(kP_POWER_PER_MM, kI_POWER_PER_MM_SEC, kD_POWER_PER_MM_PER_SEC);
    public final PIDController rDriveController = new PIDController(2 * Math.PI / 3, kI_POWER_PER_MM_SEC, kD_POWER_PER_MM_PER_SEC);
    Limelight3A limelight;
    private double maxTranslationPower = 0.75;
    private double maxRotationPower = 0.60;
    private double positionToleranceMm = 20.0;
    private double headingToleranceDeg = 5.0;
    private double settleTimeMs = 200.0;
    private double timeoutMs = 5000.0;

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

    public boolean rotateTo(double targetRad) {
        return rotateTo(targetRad, false);
    }

    public boolean rotateTo(double targetRad, boolean debug) {
        double current = getHeadingRad();

        double dRot = targetRad - current;

        if (Math.abs(dRot) < rotateToThreshold) {
            drive.setDrivePowers(0, 0, 0, 0);
            return true;
        }

//        double rotate = rDriveController.calculate(AngleUnit.normalizeRadians(targetRad + Math.PI), current);
        double rotate = rDriveController.calculate(AngleUnit.normalizeRadians(dRot), 0, 0);

        double fl = -rotate;
        double fr = rotate;
        double bl = -rotate;
        double br = rotate;

        // denominator
        double d = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br))));

        if (debug) {
            telemetry.addData("delta rotation", AngleUnit.normalizeRadians(dRot));
            telemetry.addData("rotate", rotate);

            telemetry.addData("fl", fl);
            telemetry.addData("fr", fr);
            telemetry.addData("bl", bl);
            telemetry.addData("br", br);
        }

        drive.setDrivePowers(fl / d,fr / d, bl / d, br / d);
        return false;
    }

    public boolean moveTo(double targetX, double targetY) {
        return moveTo(targetX, targetY, false);
    }

    /**
    * @return state of completion (with accuracy of moveToThreshold in millimeters
    * */
    public boolean moveTo(double targetX, double targetY, boolean debug) {
//        Pose2D pos = pinpoint.getPosition();
//
//        double currentX = pos.getX(DistanceUnit.MM);
//        double currentY = pos.getY(DistanceUnit.MM);
//
//        double fieldDX = targetX - currentX;
//        double fieldDY = targetY - currentY;
//        double heading = getHeadingRad();
//
//
//        // may have to flip these if driving is inverted
////        double normalizedX = fieldDX * Math.cos(heading) + fieldDY * Math.sin(heading);
////        double normalizedX = fieldDX * Math.cos(heading) + fieldDY * Math.sin(heading);
////        double normalizedY = -fieldDX * Math.sin(heading) + fieldDY * Math.cos(heading);
////        double normalizedY = -fieldDX * Math.sin(heading) + fieldDY * Math.cos(heading);
//
////        double normalizedX = fieldDX * Math.cos(heading)
////                + fieldDY * Math.sin(heading);
////
////        double normalizedY = -fieldDX * Math.sin(heading)
////                + fieldDY * Math.cos(heading);
//
//        // correct math?
////        double normalizedX = fieldDX * Math.cos(heading) + fieldDY * Math.sin(heading);
////        double normalizedY = -fieldDX * Math.sin(heading) + fieldDY * Math.cos(heading);
//
//        double normalizedX = targetX * Math.cos(heading) + targetY * Math.sin(heading);
//        double normalizedY = -targetX * Math.sin(heading) + targetY * Math.cos(heading);
//
//        double dist = Math.hypot(fieldDX, fieldDY);
//
//        if (dist < moveToThreshold) {
//            drive.setDrivePowers(0, 0, 0, 0);
//            return true;
//        }
//
////        double strafe = yDriveController.calculate(normalizedX, currentX);
//        double strafe = yDriveController.calculate(normalizedY, currentY);
//        double forward = xDriveController.calculate(normalizedX, targetX);
//
////        double strafe = 0;
////        double forward = xDriveController.calculate(normalizedY, currentY);
////        double forward = 0;
//
//        double fl = forward - strafe;
//        double fr = forward + strafe;
//        double bl = forward + strafe;
//        double br = forward - strafe;
//
//        // denominator
//        double d = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br))));
//        d = MathHelper.clamp(d, 0, 1);
//
//        if (Math.abs(fieldDX) < moveToThreshold && Math.abs(fieldDY) < moveToThreshold) {
//            return true;
//        }
//
//        if (debug) {
//            telemetry.addData("delta x", fieldDX);
//            telemetry.addData("delta y", fieldDY);
//            telemetry.addData("strafe", strafe);
//            telemetry.addData("forward", forward);
//
//            telemetry.addData("fl", fl);
//            telemetry.addData("fr", fr);
//            telemetry.addData("bl", bl);
//            telemetry.addData("br", br);
//        }
//
//        drive.setDrivePowers(fl / d,fr / d, bl / d, br / d);
        return false;
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
//        Pose2D pos = pinpoint.getPosition();
//
//        double currentX = pos.getX(DistanceUnit.MM);
//        double currentY = pos.getY(DistanceUnit.MM);
//        double currentRad = getHeadingRad();
//
//        double fieldDX = targetX - currentX;
//        double fieldDY = targetY - currentY;
//
//        double dist = Math.hypot(fieldDX, fieldDY);
//
//        double normalizedX = fieldDX * Math.cos(currentRad) + fieldDY * Math.sin(currentRad);
//        double normalizedY = -fieldDX * Math.sin(currentRad) + fieldDY * Math.cos(currentRad);
//
//        double dRot = AngleUnit.normalizeRadians(targetRad - currentRad);
//
//        if (dist < moveToThreshold && Math.abs(dRot) < rotateToThreshold) {
//            drive.setDrivePowers(0, 0, 0, 0);
//            return true;
//        }
//
//        double strafe = yDriveController.calculate(normalizedY, 0);
//        double forward = xDriveController.calculate(normalizedX, 0);
//
//        double rotate = rDriveController.calculate(dRot, 0);
//
//        double fl = forward - strafe - rotate;
//        double fr = forward + strafe + rotate;
//        double bl = forward + strafe - rotate;
//        double br = forward - strafe + rotate;
//
//        double d = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br))));
//
//        if (debug) {
//            telemetry.addData("current X", currentX);
//            telemetry.addData("current Y", currentY);
//
//            telemetry.addData("field dX", fieldDX);
//            telemetry.addData("field dY", fieldDY);
//
//            telemetry.addData("robot dX", normalizedX);
//            telemetry.addData("robot dY", normalizedY);
//
//            telemetry.addData("distance", dist);
//
//            telemetry.addData("current heading", currentRad);
//            telemetry.addData("target heading", targetRad);
//            telemetry.addData("delta rotation", dRot);
//
//            telemetry.addData("forward", forward);
//            telemetry.addData("strafe", strafe);
//            telemetry.addData("rotate", rotate);
//
//            telemetry.addData("fl", fl);
//            telemetry.addData("fr", fr);
//            telemetry.addData("bl", bl);
//            telemetry.addData("br", br);
//        }
//
//        drive.setDrivePowers(fl / d, fr / d, bl / d, br / d);
//
//        return false;

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

//            return new MoveToResult(
//                    currentPose, targetPose,
//                    fieldXError, fieldYError,
//                    robotForwardError, robotLeftError,
//                    headingError,
//                    DriveSignal.ZERO, WheelPowers.ZERO, true);
            return true;
        }

        telemetry.addData("current x", getX());
        telemetry.addData("current y", getY());
        telemetry.addData("current heading", getHeadingDeg());

        double controllerDt = Range.clip(dtSeconds, 1e-4, .1);
        double forwardPower = yDriveController.calculate(robotForwardError, 0.0, controllerDt);
        double leftPower = xDriveController.calculate(robotLeftError, 0.0, controllerDt);
        double counterclockwisePower = rDriveController.calculate(headingError, 0.0, controllerDt);

        double translationPower = Math.hypot(forwardPower, leftPower);
        if (translationPower > maxTranslationPower && translationPower > 0.0) {
            double translationScale = maxTranslationPower / translationPower;
            forwardPower *= translationScale;
            leftPower *= translationScale;
        }
        counterclockwisePower = Range.clip(
                counterclockwisePower,
                maxRotationPower,
                maxRotationPower);

        DriveSignal driveSignal = new DriveSignal(
                forwardPower, leftPower, counterclockwisePower);
        WheelPowers wheelPowers = driveRobotRelative(driveSignal);
//        return new MoveToResult(
//                currentPose, targetPose,
//                fieldXError, fieldYError,
//                robotForwardError, robotLeftError,
//                headingError,
//                driveSignal, wheelPowers, false);
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
