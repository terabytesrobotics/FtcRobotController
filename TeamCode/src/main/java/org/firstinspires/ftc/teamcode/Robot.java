package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.drive.DriveSignal;
import org.firstinspires.ftc.teamcode.drive.MecanumMixer;
import org.firstinspires.ftc.teamcode.drive.WheelPowers;
import org.firstinspires.ftc.teamcode.element.BaseStaticGameElement;
import org.firstinspires.ftc.teamcode.element.StaticGameElementMappings;
import org.firstinspires.ftc.teamcode.subsystem.Collector;
import org.firstinspires.ftc.teamcode.subsystem.Drive;
import org.firstinspires.ftc.teamcode.util.AStarPathfinder;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.DashboardTelemetryWrapper;
import org.firstinspires.ftc.teamcode.util.MathHelper;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.util.data.Node;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

// https://github.com/FIRST-Tech-Challenge/FtcRobotController/blob/master/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/SensorLimelight3A.java
public class Robot {
    GoBildaPinpointDriver pinpoint;
    public Drive drive;
    public Collector collector;
    public AprilTagProcessor aprilTagProcessor;
    public AprilTagLibrary aprilTagLib;
    private VisionPortal visionPortal;
    public HardwareMap hardwareMap;
//    public Telemetry telemetry;
    public static DashboardTelemetryWrapper telemetry;
    private final int moveToThreshold = 5;
    private final double rotateToThreshold = Math.PI / 32.0;
    private final double minPunchApplication = 0.05;
    private final double maxPunchApplication = 0.2;
    private final double punch = 0.2;
    public final double kP_POWER_PER_MM = 0.006;
//    public final double kI_POWER_PER_MM_SEC = 20.0;
    public final double kI_POWER_PER_MM_SEC = 0.0;
    public final double kD_POWER_PER_MM_PER_SEC = 0.0007;
    public final PIDController xDriveController = new PIDController(kP_POWER_PER_MM, 1E-5 * kI_POWER_PER_MM_SEC, kD_POWER_PER_MM_PER_SEC);
    public final PIDController yDriveController = new PIDController(kP_POWER_PER_MM, 1E-5 * kI_POWER_PER_MM_SEC, kD_POWER_PER_MM_PER_SEC);
    public final PIDController rDriveController = new PIDController(5.4, 0, .21);
    Limelight3A limelight;
    private double maxTranslationPower = 0.9;
    private double maxRotationPower = 0.60;
    private final ElapsedTime updateTimer = new ElapsedTime();
    private double updateDTSeconds = 0;

//    private double cellSamplingMM = 30;
//    private AStarPathfinder drivePathfinder = new AStarPathfinder(20 * Constants.IN_TO_MM, 22 * Constants.IN_TO_MM, cellSamplingMM, StaticGameElementMappings.obstacleNodes(cellSamplingMM));
    private double cellSamplingMM = 60;
    private AStarPathfinder drivePathfinder = new AStarPathfinder(1, 1, cellSamplingMM, StaticGameElementMappings.obstacleNodes(cellSamplingMM));
    private Node lastPathfindGoal;
    private List<Node> lastAStarPath;
    private int pathIndex;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        drive = new Drive(hardwareMap);
        collector = new Collector(hardwareMap);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // configure pinpoint
        pinpoint.setOffsets(-100.0, -30.0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        try {
            pinpoint.resetPosAndIMU();
            Thread.sleep(500);
        } catch (Exception e) {
            e.printStackTrace();
        }

        // set the starting location
        pinpoint.setPosition(new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0));

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        this.hardwareMap = hardwareMap;
        this.telemetry = new DashboardTelemetryWrapper(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.telemetry.setMsTransmissionInterval(50);
    }

    public void update() {
//        drive.setDrivePowers(0, 0, 0, 0);

        pinpoint.update();
        telemetry.update();
        updateDTSeconds = updateTimer.seconds();
        updateTimer.reset();

        for (BaseStaticGameElement element : StaticGameElementMappings.elements) {
            telemetry.addData(element.getID(), (element.getBounds().topLeft.y - element.getBounds().bottomLeft.y));
        }
    }

    public void updateDashboard() {
        telemetry.draw(getX(), getY(), getHeadingRad());
        telemetry.drawPath(lastAStarPath);
        telemetry.updateDashboard();
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

    public void setPos(Pose2D pose) {
        pinpoint.setPosition(pose);
    }

    public void updateAprilTags() {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            telemetry.addData("Limelight", "No valid results");
            return;
        }

        List<LLResultTypes.FiducialResult> detections =
                result.getFiducialResults();

        telemetry.addData("Tags detected", detections.size());

        for (LLResultTypes.FiducialResult detection : detections) {
            telemetry.addData(
                    "Tag ID",
                    detection.getFiducialId()
            );

            telemetry.addData(
                    "Horizontal angle (deg)",
                    detection.getTargetXDegrees()
            );

            telemetry.addData(
                    "Vertical angle (deg)",
                    detection.getTargetYDegrees()
            );

            telemetry.addData(
                    "Tag area (%)",
                    detection.getTargetArea()
            );
        }
    }


    public boolean moveToRot(double targetX, double targetY, double targetRad) {
        return moveToRot(targetX, targetY, targetRad, false);
    }

    public boolean moveToRot(
            double targetX,
            double targetY,
            double targetRad,
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

        double controllerDt = Range.clip(updateDTSeconds, 1e-4, .1);
        double forwardPower = yDriveController.calculate(robotForwardError, 0.0, controllerDt);
        double leftPower = xDriveController.calculate(robotLeftError, 0.0, controllerDt);
        double hyp = Math.hypot(forwardPower, leftPower);
        double counterclockwisePower = rDriveController.calculate(headingError, 0.0, controllerDt);

        if (hyp > maxTranslationPower && hyp > 0.0) {
            double translationScale = maxTranslationPower / hyp;
            forwardPower *= translationScale;
            leftPower *= translationScale;
        }
//        // apply punch
//        if (hyp > minPunchApplication && hyp < maxPunchApplication) {
//            double scale = punch / hyp;
//            forwardPower *= scale;
//            leftPower *= scale;
//        }

        counterclockwisePower = Range.clip(
                counterclockwisePower,
                -maxRotationPower,
                maxRotationPower);

        DriveSignal driveSignal = new DriveSignal(
                forwardPower, leftPower, counterclockwisePower);
        driveRobotRelative(driveSignal);

        telemetry.addData("forward", driveSignal.getForward());
        telemetry.addData("left", driveSignal.getLeft());
        return false;
    }

    public boolean pathfindTest(double targetX, double targetY) {
        Node goal = new Node(targetX, targetY);

        boolean goalChanged = lastPathfindGoal == null ||
                        Math.abs(lastPathfindGoal.x - targetX) > 1.0 ||
                        Math.abs(lastPathfindGoal.y - targetY) > 1.0;

        if (lastAStarPath == null || lastAStarPath.isEmpty() || goalChanged) {
            pathIndex = 0;
            lastAStarPath = drivePathfinder.eval(new Node(getX(), getY()), goal);
            lastPathfindGoal = goal;
        }

        if (lastAStarPath == null || lastAStarPath.isEmpty()) {
            return false;
        }

        Node bestNode = null;
        int bestIndex = pathIndex;

        double targetDist = 80;
        int maxSearchNodes = 15;
        double traveled = 0;

        for (int i = 0; i < maxSearchNodes; i++) {
            int index = pathIndex + i;

            if (index >= lastAStarPath.size()) {
                break;
            }

            Node node = lastAStarPath.get(index);

            if (i == 0) {
                traveled = MathHelper.approxDistanceTo(getX(), getY(), node.x, node.y);
            } else {
                Node previous = lastAStarPath.get(index - 1);
                traveled += MathHelper.approxDistanceTo(previous.x, previous.y, node.x, node.y);
            }

            bestNode = node;
            bestIndex = index;

            if (traveled >= targetDist) {
                break;
            }
        }

        if (bestNode != null) {
            pathIndex = bestIndex;

            telemetry.addData("best x", bestNode.x);
            telemetry.addData("best y", bestNode.y);
            telemetry.addData("path index", pathIndex);
            telemetry.addData("path size", lastAStarPath.size());

            return moveToRot(bestNode.x, bestNode.y, 0, true);
        }

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
