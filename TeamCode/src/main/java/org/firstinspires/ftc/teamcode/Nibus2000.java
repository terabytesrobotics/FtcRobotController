package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.NibusConstants.*;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.SwitchableCameraName;
import org.firstinspires.ftc.teamcode.Processors.WindowBoxesVisionProcessor;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.AlliancePropPosition;
import org.firstinspires.ftc.teamcode.util.BlueGrabberState;
import org.firstinspires.ftc.teamcode.util.CollectorState;
import org.firstinspires.ftc.teamcode.util.GreenGrabberState;
import org.firstinspires.ftc.teamcode.util.OnActivatedEvaluator;
import org.firstinspires.ftc.teamcode.util.TrueForTime;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Queue;

public class Nibus2000 {

    private final SampleMecanumDrive drive;
    private final TouchSensor armMin;
    private final TouchSensor extenderMin;
    private final PIDController armcontrol = new PIDController(ARM_CONTROL_P, ARM_CONTROL_I, ARM_CONTROL_D);
    private final DcMotorEx armLeft;
    private final DcMotorEx armRight;
    private final DcMotorEx extender;
    private final Servo wrist;
    private final Servo greenGrabber;
    private final Servo blueGrabber;
    private final DcMotorEx launcher;

    private CollectorState collectorState = CollectorState.DRIVING_SAFE;
    private double armTickTarget;
    private double extenderTickTarget;
    private double wristPosition;
    private final OnActivatedEvaluator a1PressedEvaluator;
    private final OnActivatedEvaluator x1PressedEvaluator;
    private final OnActivatedEvaluator y1PressedEvaluator;
    private final OnActivatedEvaluator b1PressedEvaluator;
    private final OnActivatedEvaluator a2PressedEvaluator;
    private final OnActivatedEvaluator b2PressedEvaluator;
    private final OnActivatedEvaluator x2PressedEvaluator;
    private final OnActivatedEvaluator y2PressedEvaluator;
    private final OnActivatedEvaluator rb2PressedEvaluator;
    private final OnActivatedEvaluator lb2PressedEvaluator;
    private final OnActivatedEvaluator rb1PressedEvaluator;
    private final OnActivatedEvaluator lb1PressedEvaluator;


    private final OnActivatedEvaluator dpadUp2PressedEvaluator;
    private final OnActivatedEvaluator dpadDown2PressedEvaluator;
    private final OnActivatedEvaluator dpadLeft2PressedEvaluator;
    private final OnActivatedEvaluator dpadRight2PressedEvaluator;
    private final OnActivatedEvaluator dpadLeft1PressedEvaluator;
    private final OnActivatedEvaluator dpadRight1PressedEvaluator;
    private final TrueForTime settledAtPoseTarget;
    private final OnActivatedEvaluator rs1PressedEvaluator;
    private final OnActivatedEvaluator ls1PressedEvaluator;

    private final OnActivatedEvaluator onEnteredUpstageEvaluator;
    private final OnActivatedEvaluator onEnteredBackstageEvaluator;
    private BlueGrabberState blueGrabberState = BlueGrabberState.NOT_GRABBED;
    private GreenGrabberState greenGrabberState = GreenGrabberState.NOT_GRABBED;
    private final Telemetry telemetry;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;
    private final WebcamName frontCamera;
    private final WebcamName backCamera;
    private final DigitalChannel indicator1Red;
    private final DigitalChannel indicator1Green;
    private final AllianceColor allianceColor;
    private final VisionPortal visionPortal;
    private final AprilTagProcessor aprilTagProcessor;
    private final WindowBoxesVisionProcessor propFinder;
    private final RevBlinkinLedDriver blinkinLedDriver;
    private NibusState state;
    private ElapsedTime timeSinceStart;
    private ElapsedTime timeInState;
    private Pose2d latestPoseEstimate = null;
    private boolean launchingAirplane = false;
    private int launchingAirplaneTimeMillis = 0;
    private int endgameLiftStage = 0;
    private NibusAutonomousPlan autonomousPlan = null;
    private final ElapsedTime currentCommandTime = new ElapsedTime();
    private final ElapsedTime currentCommandSettledTime = new ElapsedTime();
    private NibusCommand currentCommand = null;
    private final ArrayList<NibusCommand> commandSequence = new ArrayList<>();
    private NibusState continuationState = null;
    private int framesProcessed = 0;
    private final int[] leftMidRightVotes = new int[] { 0, 0, 0 };
    private AlliancePropPosition alliancePropPosition = null;
    private final ElapsedTime elapsedPropFrameTime = new ElapsedTime();
    private NibusApproach nextNibusApproach = null;
    private ElapsedTime approachSettlingTimer = null;
    private Pose2d lastAprilTagFieldPosition = null;
    private double lastAprilTagFieldPositionMillis = 0;
    private Pose2d poseTarget = null;
    private NibusApproach approachTarget = null;
    private final Queue<Pose2d> poseQueue = new LinkedList<>();
    private double greenGrabberManualOffset = 0;
    private double blueGrabberManualOffset = 0;
    private Pose2d pointOfInterest = null;
    private final Map<CollectorState, Integer> armNudgesPerPosition = new HashMap<CollectorState, Integer>();
    private final Map<CollectorState, Integer> wristNudgesPerPosition = new HashMap<CollectorState, Integer>();
    private final Map<CollectorState, Integer> extenderNudgesPerPosition = new HashMap<CollectorState, Integer>();
    private double scoringHeightOffset = 0.0;
    private final double RAISE_RATE_INCH_PER_MILLI = 4.0 / 1000;

    private RevColorSensorV3 colorSensorGreen;

    private RevColorSensorV3 colorSensorBlue;
    private double pixelMinGrab = 13;
    private double colorSensorMaxProx = 15;
    private int millsBeforeExtract = 300;

    private boolean grabGreen = false;
    private boolean grabBlue = false;
    private boolean armedBlue = false;
    private boolean armedGreen = false;
    private boolean retract = false;
    private double grabTime = 0;

    public Nibus2000(AllianceColor allianceColor, Gamepad gamepad1, Gamepad gamepad2, HardwareMap hardwareMap, Telemetry telemetry) {
        this.allianceColor = allianceColor;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;
        this.state = NibusState.MANUAL_DRIVE;

        frontCamera = hardwareMap.get(WebcamName.class, "Webcam 1");
        backCamera = hardwareMap.get(WebcamName.class, "Webcam 2");
        SwitchableCameraName switchableCamera = ClassFactory.getInstance().getCameraManager().nameForSwitchableCamera(frontCamera, backCamera);
        aprilTagProcessor = new AprilTagProcessor.Builder().build();
        propFinder = new WindowBoxesVisionProcessor();
        visionPortal = new VisionPortal.Builder()
                .setCamera(switchableCamera)
                .addProcessor(propFinder)
                .addProcessor(aprilTagProcessor)
                .build();

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        indicator1Red = hardwareMap.get(DigitalChannel.class, "indicator1red");
        indicator1Green = hardwareMap.get(DigitalChannel.class, "indicator1green");
        indicator1Red.setMode(DigitalChannel.Mode.OUTPUT);
        indicator1Green.setMode(DigitalChannel.Mode.OUTPUT);

        settledAtPoseTarget = new TrueForTime(APPROACH_SETTLE_TIME_MS, this::isAtPoseTarget);
        a1PressedEvaluator = new OnActivatedEvaluator(() -> gamepad1.a);
        b1PressedEvaluator = new OnActivatedEvaluator(() -> gamepad1.b);
        x1PressedEvaluator = new OnActivatedEvaluator(() -> gamepad1.x);
        y1PressedEvaluator = new OnActivatedEvaluator(() -> gamepad1.y);
        rs1PressedEvaluator = new OnActivatedEvaluator(() -> gamepad1.right_stick_button);
        ls1PressedEvaluator = new OnActivatedEvaluator(() -> gamepad1.left_stick_button);
        a2PressedEvaluator = new OnActivatedEvaluator(() -> gamepad2.a);
        b2PressedEvaluator = new OnActivatedEvaluator(() -> gamepad2.b);
        x2PressedEvaluator = new OnActivatedEvaluator(() -> gamepad2.x);
        y2PressedEvaluator = new OnActivatedEvaluator(() -> gamepad2.y);
        rb2PressedEvaluator = new OnActivatedEvaluator(() -> gamepad2.right_bumper);
        lb2PressedEvaluator = new OnActivatedEvaluator(()->gamepad2.left_bumper);
        rb1PressedEvaluator = new OnActivatedEvaluator(() -> gamepad1.right_bumper);
        lb1PressedEvaluator = new OnActivatedEvaluator(()->gamepad1.left_bumper);

        dpadUp2PressedEvaluator = new OnActivatedEvaluator(() -> gamepad2.dpad_up);
        dpadDown2PressedEvaluator = new OnActivatedEvaluator(() -> gamepad2.dpad_down);
        dpadLeft2PressedEvaluator = new OnActivatedEvaluator(() -> gamepad2.dpad_left);
        dpadRight2PressedEvaluator = new OnActivatedEvaluator(() -> gamepad2.dpad_right);
        dpadLeft1PressedEvaluator = new OnActivatedEvaluator(() -> gamepad1.dpad_left);
        dpadRight1PressedEvaluator = new OnActivatedEvaluator(() -> gamepad1.dpad_right);
        onEnteredUpstageEvaluator = new OnActivatedEvaluator(this::isUpstage);
        onEnteredBackstageEvaluator = new OnActivatedEvaluator(this::isBackstage);

        armcontrol.setTolerance(ARM_TOLERANCE);
        armMin = hardwareMap.get(TouchSensor.class, "armMin1");
        extenderMin = hardwareMap.get(TouchSensor.class, "extenderMin3");
        armLeft = hardwareMap.get(DcMotorEx.class, "armE0");
        armRight = hardwareMap.get(DcMotorEx.class, "armE3");
        extender = hardwareMap.get(DcMotorEx.class, "extenderE1");
        wrist = hardwareMap.get(Servo.class, "redE3");
        greenGrabber = hardwareMap.get(Servo.class, "greenE0");
        blueGrabber = hardwareMap.get(Servo.class, "blueE1");

        launcher = hardwareMap.get(DcMotorEx.class, "launcherE2");
        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcher.setDirection(DcMotorSimple.Direction.FORWARD);

        colorSensorBlue = hardwareMap.get(RevColorSensorV3.class,"colorBlue");
        colorSensorGreen = hardwareMap.get(RevColorSensorV3.class,"colorGreen");

        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN);

        for (CollectorState state : CollectorState.values()) {
            armNudgesPerPosition.put(state, 0);
            wristNudgesPerPosition.put(state, 0);
            extenderNudgesPerPosition.put(state, 0);
        }
    }

    private void runTelemetry() {
        if (lastAprilTagFieldPosition != null) {
            telemetry.addData("estimate-x", lastAprilTagFieldPosition.getX());
            telemetry.addData("estimate-y", lastAprilTagFieldPosition.getY());
            telemetry.addData("etimate-heading", lastAprilTagFieldPosition.getHeading());
        }
        telemetry.addData("x", latestPoseEstimate.getX());
        telemetry.addData("y", latestPoseEstimate.getY());
        telemetry.addData("heading", latestPoseEstimate.getHeading());
        telemetry.update();
    }

    private void setPoseEstimate(Pose2d pose) {
        Pose2d delta = null;
        if (latestPoseEstimate != null) {
            delta = pose.minus(latestPoseEstimate);
        }

        // In the rare case we get our first april tag estimate while in point of interest mode,
        // don't go haywire, update the point of interest with the delta.
        if (pointOfInterest != null && lastAprilTagFieldPosition == null && delta != null) {
            pointOfInterest = pointOfInterest.plus(delta);
        }

        drive.setPoseEstimate(pose);
    }

    public void autonomousInit(Pose2d startPose, NibusAutonomousPlan autonomousPlan) {
        this.autonomousPlan = autonomousPlan;
        setPoseEstimate(startPose);

        wrist.setPosition(.25);
        sleep(1000);
        extenderInitSequence(true);
        armInitSequence(true);
        wrist.setPosition(.25);
        grabberInit();
        visionPortal.setActiveCamera(backCamera);
        visionPortal.setProcessorEnabled(propFinder, true);
        visionPortal.setProcessorEnabled(aprilTagProcessor, false);
    }

    public void teleopInit() {
        setPoseEstimate(new Pose2d());
        wrist.setPosition(.25);

        extenderInitSequence(false);
        armInitSequence(false);

        visionPortal.setActiveCamera(frontCamera);
        visionPortal.setProcessorEnabled(propFinder, false);
        visionPortal.setProcessorEnabled(aprilTagProcessor, true);
    }

    private void zeroArmMotors() {
        armLeft.setPower(0);
        armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        armLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armRight.setPower(0);
        armRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRight.setDirection(DcMotorSimple.Direction.FORWARD);
        armRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void zeroExtender() {
        extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extender.setTargetPosition(0);
        extender.setTargetPositionTolerance(EXTENDER_TICK_TOLERANCE);
        extender.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        extender.setPower(EXTENDER_POWER);
    }

    public void startup(NibusState startupState) {
        timeSinceStart = new ElapsedTime();
        timeInState = new ElapsedTime();
        state = startupState;
        elapsedPropFrameTime.reset();
    }

    private double lastCycleMillis = 0;
    private double dtMillis = 0;

    public boolean evaluate() {
        if (lastCycleMillis != 0) {
            dtMillis = timeSinceStart.milliseconds() - lastCycleMillis;
        }
        lastCycleMillis = timeSinceStart.milliseconds();
        drive.update();
        latestPoseEstimate = drive.getPoseEstimate();

        evaluateIndicatorLights();
        evaluatePositioningSystems();
        controlScoringSystems();

        NibusState currentState = state;
        NibusState nextState = state;
        switch (state) {
            case MANUAL_DRIVE:
                nextState = evaluateDrivingAndScoring();
                break;
            case DRIVE_DIRECT_TO_POSE:
                nextState = evaluateDriveDirectToPosition();
                break;
            case AUTONOMOUSLY_DRIVING:
                nextState = evaluateDrivingAutonomously();
                break;
            case DETECT_ALLIANCE_MARKER:
                nextState = evaluateDetectAllianceMarker();
                break;
            case STOPPED_UNTIL_END:
                // Hold the drive still.
                drive.setWeightedDrivePower(new Pose2d());
            default:
                break;
        }
        if (nextState != currentState) {
            timeInState.reset();
            state = nextState;
        }

        runTelemetry();

        return state != NibusState.HALT_OPMODE;
    }

    private void evaluateIndicatorLights() {

        double timeSincePositionSet = (timeSinceStart.milliseconds() - lastAprilTagFieldPositionMillis);
        if (lastAprilTagFieldPosition != null && timeSincePositionSet < POSITION_ACQUIRED_INDICATE_MILLIS) {
            if (timeSincePositionSet % (2 * POSITION_ACQUIRED_PULSE_MILLIS) > POSITION_ACQUIRED_PULSE_MILLIS) {
                indicator1Green.setState(true);
                indicator1Red.setState(true);
            } else {
                indicator1Green.setState(false);
                indicator1Red.setState(false);
            }
        }

        if (hasPositionEstimate()) {
            indicator1Green.setState(true);
            indicator1Red.setState(false);
        } else {
            indicator1Green.setState(false);
            indicator1Red.setState(false);
        }

        if (pointOfInterest == null) {
            blinkinLedDriver.setPattern(allianceColor.getAllianceColorBlinkinPattern());
        } else {
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_STROBE);
        }
    }

    private void evaluatePositioningSystems() {
        boolean usingBackCamera = visionPortal.getActiveCamera().equals(backCamera);
        double cameraDistanceOffset = usingBackCamera ? BACK_CAMERA_OFFSET_INCHES : FRONT_CAMERA_OFFSET_INCHES;
        double cameraAngleOffset = usingBackCamera ? Math.PI : 0;

        List<AprilTagDetection> detections = aprilTagProcessor.getFreshDetections();
        if (detections != null) {
            for (AprilTagDetection detection : detections) {
                // This sometimes happens.
                if (detection.ftcPose == null) {
                    continue;
                }

                if (detection.ftcPose.range > APRIL_TAG_RECOGNITION_MAX_RANGE ||
                        detection.ftcPose.range < APRIL_TAG_RECOGNITION_MIN_RANGE ||
                        Math.abs(Math.toRadians(detection.ftcPose.bearing)) > APRIL_TAG_RECOGNITION_BEARING_THRESHOLD ||
                        Math.abs(Math.toRadians(detection.ftcPose.yaw)) > APRIL_TAG_RECOGNITION_YAW_THRESHOLD) {
                    continue;
                }

                Pose2d estimatedPose = calculateRobotPose(detection, cameraDistanceOffset, cameraAngleOffset);
                if (poseQueue.size() >= APRIL_TAG_QUEUE_CAPACITY) {
                    poseQueue.poll(); // Remove the oldest element
                }
                poseQueue.offer(estimatedPose);
            }
        }

        // Calculate average and variance
        if (poseQueue.size() == APRIL_TAG_QUEUE_CAPACITY) {
            Pose2d averagePose = calculateAveragePose(poseQueue);
            Pose2d variancePose = calculateVariancePose(poseQueue, averagePose);

            // Thresholds for variance, adjust as needed
            double translationVarianceThreshold = 1.0;
            double headingVarianceThreshold = Math.PI / 32;
            if (variancePose.getX() <= translationVarianceThreshold && variancePose.getY() <= translationVarianceThreshold && variancePose.getHeading() <= headingVarianceThreshold) {
                setPoseEstimate(averagePose);
                lastAprilTagFieldPosition = averagePose;
                lastAprilTagFieldPositionMillis = timeSinceStart.milliseconds();
                poseQueue.clear();
            }
        }
    }

    private Pose2d calculateAveragePose(Queue<Pose2d> poses) {
        double sumX = 0, sumY = 0, sumHeading = 0;
        for (Pose2d pose : poses) {
            sumX += pose.getX();
            sumY += pose.getY();
            sumHeading += pose.getHeading();
        }
        int count = poses.size();
        return new Pose2d(sumX / count, sumY / count, Angle.norm(sumHeading / count));
    }

    private Pose2d calculateVariancePose(Queue<Pose2d> poses, Pose2d averagePose) {
        double varianceX = 0, varianceY = 0, varianceHeading = 0;
        for (Pose2d pose : poses) {
            varianceX += Math.pow(pose.getX() - averagePose.getX(), 2);
            varianceY += Math.pow(pose.getY() - averagePose.getY(), 2);
            varianceHeading += Math.pow(Angle.normDelta(pose.getHeading() - averagePose.getHeading()), 2);
        }
        int count = poses.size();
        return new Pose2d(Math.sqrt(varianceX / count), Math.sqrt(varianceY / count), Math.sqrt(varianceHeading / count));
    }

    private NibusState evaluateDrivingAndScoring() {
        if (lastAprilTagFieldPosition == null) {
            if (gamepad1.dpad_up) {
                setPoseEstimate(new Pose2d(0, 0, allianceColor.OperatorHeadingOffset));
            }
        }

        // Toggle collect mode.
        boolean isCollecting = collectorState == CollectorState.COLLECTION;
        if (a1PressedEvaluator.evaluate() || a2PressedEvaluator.evaluate()) {
            if (isCollecting) {
                setCollectorState(CollectorState.DRIVING_SAFE);
            } else {
                setCollectorState(CollectorState.COLLECTION);
            }
        }


        boolean isManualArmControl = collectorState == null;
        if (b1PressedEvaluator.evaluate() || b2PressedEvaluator.evaluate()) {
            if (isManualArmControl) {
                setCollectorState(CollectorState.DRIVING_SAFE);
            } else {
                setManualArmState();
            }
        } else if (isManualArmControl) {
            double SCORING_HEIGHT_MAX = 10;
            scoringHeightOffset += (-gamepad2.right_stick_y) * dtMillis * RAISE_RATE_INCH_PER_MILLI;
            scoringHeightOffset = Math.max(0, Math.min(SCORING_HEIGHT_MAX, scoringHeightOffset));
            setManualArmState();
        } else {
            setCollectorState(collectorState);
        }


        //***NEW CODE FOR AUTO GRABBER***
        //Evaluate Green Color sensor to see if the pixel is in grabbing range
        if(colorSensorGreen.getDistance(DistanceUnit.MM) <= pixelMinGrab){
            grabGreen = true;
        }else grabGreen = false;
        telemetry.addData("Grab Green",grabGreen);
        telemetry.addData("GreenProx",colorSensorGreen.getDistance(DistanceUnit.MM));
        //Evaluate blue color sensor to se if pixel is in grabbing range
        if(colorSensorBlue.getDistance(DistanceUnit.MM) <= pixelMinGrab){
            grabBlue = true;
        }else grabBlue = false;
        telemetry.addData("Blue grab",grabBlue);
        telemetry.addData("BlueProx",colorSensorBlue.getDistance(DistanceUnit.MM));

        //Mapping LB and RB to arm auto grabbers
        if(!armedBlue&& lb1PressedEvaluator.evaluate()) armedBlue = true;
        telemetry.addData("armed BLue",armedBlue);
        if(armedBlue && lb1PressedEvaluator.evaluate()){
            armedBlue = false;
            blueGrabberState = BlueGrabberState.GRABBED;
            if(!armedGreen){

                retract = true;
                grabTime = System.currentTimeMillis();

                //collectorState = CollectorState.DRIVING_SAFE;
            }
        }
        telemetry.addData("rb new click",rb1PressedEvaluator.isNewClick());
        telemetry.addData("lb new click",lb1PressedEvaluator.isNewClick());

        telemetry.addData("armed Green",armedGreen);
        if(!armedGreen && rb1PressedEvaluator.evaluate()){
            armedGreen = true;
        }
        if(armedGreen && rb1PressedEvaluator.evaluate()) {
            armedGreen = false;
            greenGrabberState = GreenGrabberState.GRABBED;
            if(!armedBlue){

                retract = true;
                grabTime = System.currentTimeMillis();

                //collectorState = CollectorState.DRIVING_SAFE;
            }
        }
        if(gamepad1.x){ armedBlue = false; armedGreen = false;}

        //Auto grabber logic Blue
        if(armedBlue) {
            if(grabBlue) {
                blueGrabberState = BlueGrabberState.GRABBED;

                armedBlue = false;
                if(!armedGreen){

                    retract = true;
                    grabTime = System.currentTimeMillis();

                    //collectorState = CollectorState.DRIVING_SAFE;
                    }

            }else{
                collectorState = CollectorState.COLLECTION;
                blueGrabberState = BlueGrabberState.NOT_GRABBED;
            }
        }


        //Auto grabber logic Green
        if(armedGreen) {
            if(grabGreen) {
                greenGrabberState = GreenGrabberState.GRABBED;
                armedGreen = false;
                if(!armedBlue){
                    retract = true;
                    grabTime = System.currentTimeMillis();
                    //collectorState = CollectorState.DRIVING_SAFE;
                }
            }else{
                collectorState = CollectorState.COLLECTION;
                greenGrabberState = GreenGrabberState.NOT_GRABBED;
            }
        }

        telemetry.addData("Time Elapsed", System.currentTimeMillis()-grabTime);
        telemetry.addData("time",System.currentTimeMillis());
        if(retract && System.currentTimeMillis() -grabTime >= millsBeforeExtract) {
            collectorState = CollectorState.DRIVING_SAFE;
            retract = false;
        }

        if(armedBlue|| armedGreen){
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_RED);
        }else {
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_BEATS_PER_MINUTE);
        }











        if (y1PressedEvaluator.evaluate() || y2PressedEvaluator.evaluate()) {
            if (pointOfInterest == null) {
                pointOfInterest = latestPoseEstimate;
            } else {
                pointOfInterest = null;
            }
        }

        if (rs1PressedEvaluator.evaluate()) {
            endgameLiftStage = Math.min(3, endgameLiftStage + 1);
            setCollectorState(endgameLiftStage(endgameLiftStage));
        } else if (ls1PressedEvaluator.evaluate()) {
            endgameLiftStage = Math.max(0, endgameLiftStage - 1);
            setCollectorState(endgameLiftStage(endgameLiftStage));
        }

        if (x2PressedEvaluator.evaluate()) {
            setCollectorState(CollectorState.DRIVING_SAFE);
            pointOfInterest = null;
        }

        Pose2d controlPose = new Pose2d();
        if (pointOfInterest == null) {
            if (gamepad2.left_bumper || gamepad2.right_bumper) {
                double focalPointYOffset = 0;
                if (gamepad2.left_bumper) {
                    focalPointYOffset += 5.5;
                }
                if (gamepad2.right_bumper) {
                    focalPointYOffset -= 5.5;
                }

                focalPointYOffset += (-gamepad2.right_stick_x * 3);
                double focalPointXOffset = (-gamepad2.left_stick_y * 4);
                double collectorHeadOrthogonalOffset = (-gamepad2.left_stick_x * 2);

                Pose2d scoringFocalPoint =
                        allianceColor.getAprilTagForScoringPosition(CenterStageBackdropPosition.CENTER)
                                .facingPose()
                                .plus(new Pose2d(focalPointXOffset, focalPointYOffset, 0));

                Pose2d robotPoseForFocalPoint = NibusHelpers.robotPose(scoringFocalPoint, 20, collectorHeadOrthogonalOffset);

                telemetry.addData("scoringFocalPoint", "x: %5.1f, y: %5.1f, th: %5.1f", scoringFocalPoint.getX(), scoringFocalPoint.getY(), scoringFocalPoint.getHeading());
                telemetry.addData("robotPoseForFocalPoint", "x: %5.1f, y: %5.1f, th: %5.1f", robotPoseForFocalPoint.getX(), robotPoseForFocalPoint.getY(), robotPoseForFocalPoint.getHeading());
                controlPose = getPoseTargetAutoDriveControl(robotPoseForFocalPoint);
            } else {
                controlPose = getScaledHeadlessDriverInput(gamepad1);

                if (hasPositionEstimate() && gamepad1.x) {
                    int closestLaneY = CenterStageConstants.getClosestLane(latestPoseEstimate.getY());
                    double errorY = closestLaneY - latestPoseEstimate.getY();
                    double errorHeading = Angle.normDelta(0 - latestPoseEstimate.getHeading());

                    double laneLockY = Range.clip(errorY * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                    double laneLockRotation = Range.clip(errorHeading * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                    controlPose = controlPose.plus(new Pose2d(0, laneLockY, laneLockRotation));
                }
            }

        } else if (collectorState == CollectorState.COLLECTION) {
            Pose2d driver2Input = new Pose2d(
                    -gamepad2.left_stick_y * -1,
                    -gamepad2.left_stick_x * -1,
                    -gamepad2.right_stick_x);
            controlPose =
                    driver2Input.div(1)
                            .plus(getPoseTargetAutoDriveControl(pointOfInterest));
        }

        drive.setWeightedDrivePower(controlPose);

        //greenGrabberState = GreenGrabberState.GRABBED;
        //blueGrabberState = BlueGrabberState.GRABBED;
        double blueGrabberActuationRange =
                blueGrabberState.toggle().ServoPosition - blueGrabberState.ServoPosition;
        double greenGrabberActuationRange =
                greenGrabberState.toggle().ServoPosition - greenGrabberState.ServoPosition;
        blueGrabberManualOffset = gamepad2.left_trigger * blueGrabberActuationRange;
        greenGrabberManualOffset = gamepad2.right_trigger * greenGrabberActuationRange;

        telemetry.addData("Collector state", collectorState);
        telemetry.addData("Wrist pos:",wrist.getPosition());

        if (gamepad1.left_bumper && gamepad1.right_bumper) {
            launchingAirplane = true;
            launchingAirplaneTimeMillis = (int) timeSinceStart.milliseconds();
        }

        evaluateNudgeControls();

        return NibusState.MANUAL_DRIVE;
    }

    private void evaluateNudgeControls() {
        Integer armNudgeCountBoxed = armNudgesPerPosition.getOrDefault(collectorState, 0);
        Integer wristNudgeCountBoxed = wristNudgesPerPosition.getOrDefault(collectorState, 0);
        Integer extenderNudgeCountBoxed = extenderNudgesPerPosition.getOrDefault(collectorState, 0);
        int armNudgeCount = armNudgeCountBoxed == null ? 0 : armNudgeCountBoxed;
        int wristNudgeCount = wristNudgeCountBoxed == null ? 0 : wristNudgeCountBoxed;
        int extenderNudgeCount = extenderNudgeCountBoxed == null ? 0 : extenderNudgeCountBoxed;

        if (dpadUp2PressedEvaluator.evaluate()) {
            armNudgesPerPosition.put(collectorState,
                    Math.min(armNudgeCount + 1, ARM_MAX_TRIM_INCREMENTS));
        }

        if (dpadDown2PressedEvaluator.evaluate()) {
            armNudgesPerPosition.put(collectorState,
                    Math.max(armNudgeCount - 1, -ARM_MAX_TRIM_INCREMENTS));
        }

        if (dpadLeft2PressedEvaluator.evaluate()) {
            wristNudgesPerPosition.put(collectorState,
                    Math.min(wristNudgeCount + 1, WRIST_MAX_TRIM_INCREMENTS));
        }

        if (dpadRight2PressedEvaluator.evaluate()) {
            wristNudgesPerPosition.put(collectorState,
                    Math.max(wristNudgeCount - 1, -WRIST_MAX_TRIM_INCREMENTS));
        }

        if (dpadRight1PressedEvaluator.evaluate()) {
            extenderNudgesPerPosition.put(collectorState,
                    Math.min(extenderNudgeCount + 1, 100));
        }

        if (dpadLeft1PressedEvaluator.evaluate()) {
            extenderNudgesPerPosition.put(collectorState,
                    Math.max(extenderNudgeCount - 1, -100));
        }
    }

    private CollectorState endgameLiftStage(int stageNumber) {
        switch (stageNumber) {
            case 1:
                return CollectorState.HANG1;
            case 2:
                return CollectorState.HANG2;
            case 3:
                return CollectorState.HANG3;
            case 0:
            default:
                return CollectorState.DRIVING_SAFE;
        }
    }

    private boolean isAtPoseTarget() {
        return isAtPoseTarget(poseTarget);
    }

    private boolean isAtPoseTarget(Pose2d target) {
        Pose2d error = getPoseTargetError(target);
        if (latestPoseEstimate == null || error == null) return false;
        return Math.hypot(error.getX(), error.getY()) < DRIVE_TO_POSE_THRESHOLD &&
                Math.abs(error.getHeading()) < TURN_ERROR_THRESHOLD;
    }

    private Pose2d getPoseTargetError(Pose2d poseTarget) {
        if (latestPoseEstimate == null) return null;
        return new Pose2d(poseTarget.getX() - latestPoseEstimate.getX(),
                poseTarget.getY() - latestPoseEstimate.getY(),
                Angle.normDelta(poseTarget.getHeading() - latestPoseEstimate.getHeading()));
    }

    private Pose2d getPoseTargetAutoDriveControl(Pose2d poseTarget) {
        Pose2d error = getPoseTargetError(poseTarget);
        if (latestPoseEstimate == null || error == null) return new Pose2d();

        double robotHeading = latestPoseEstimate.getHeading();

        double translationErrorMagnitude = Math.hypot(error.getX(), error.getY());
        double translationErrorFieldHeading = Math.atan2(error.getY(), error.getX());

        double adjustedHeading = translationErrorFieldHeading - robotHeading;
        double robotXErrorMagnitude = translationErrorMagnitude * Math.cos(adjustedHeading);
        double robotYErrorMagnitude = translationErrorMagnitude * Math.sin(adjustedHeading);

        return new Pose2d(
                Range.clip(robotXErrorMagnitude * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED),
                Range.clip(robotYErrorMagnitude * SPEED_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE),
                Range.clip(error.getHeading() * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN));
    }

    private NibusState evaluateDriveDirectToPosition() {
        if (y1PressedEvaluator.evaluate() || poseTarget == null) {
            approachSettlingTimer = null;
            return NibusState.MANUAL_DRIVE;
        }

        drive.setWeightedDrivePower(getPoseTargetAutoDriveControl(poseTarget));

        if (settledAtPoseTarget.evaluate()) {
            poseTarget = null;
            return NibusState.MANUAL_DRIVE;
        }

        return NibusState.DRIVE_DIRECT_TO_POSE;
    }

    private NibusState evaluateDrivingAutonomously() {
        final int SETTLE_TIME_MILLIS = 250;

        if (commandSequence.size() == 0) {
            NibusState _continuationState = continuationState;
            continuationState = null;
            currentCommandTime.reset();
            currentCommandSettledTime.reset();
            return _continuationState == null ? NibusState.MANUAL_DRIVE : _continuationState;
        }

        if (currentCommand == null) {
            currentCommand = commandSequence.get(0);
            currentCommandTime.reset();
            currentCommandSettledTime.reset();

            if (currentCommand.BlueGrabberState != null) {
                blueGrabberState = currentCommand.BlueGrabberState;
            }

            if (currentCommand.GreenGrabberState != null) {
                greenGrabberState = currentCommand.GreenGrabberState;
            }

            if (currentCommand.CollectorState != null) {
                Log.d("evaluateDrivingAutonomously", "Setting collector state");
                setCollectorState(currentCommand.CollectorState);
            }

            if (currentCommand.TrajectoryCreator != null) {
                Log.d("evaluateDrivingAutonomously", "Following trajectory async");
                drive.followTrajectoryAsync(currentCommand.TrajectoryCreator.create());
            }
        }

        if (currentCommand.DriveDirectToPose != null) {
            drive.setWeightedDrivePower(
                    getPoseTargetAutoDriveControl(currentCommand.DriveDirectToPose));
        }

        boolean collectorSettled = currentCommand.CollectorState == null || armAndExtenderSettled();
        boolean trajectoryCompleted = currentCommand.TrajectoryCreator == null || !drive.isBusy();
        boolean driveCompleted = currentCommand.DriveDirectToPose == null || isAtPoseTarget(currentCommand.DriveDirectToPose);
        boolean settledRightNow = collectorSettled && trajectoryCompleted && driveCompleted;

        boolean minTimeElapsed = currentCommandTime.milliseconds() > SETTLE_TIME_MILLIS;
        if (!settledRightNow) {
            currentCommandSettledTime.reset();
        } else if (minTimeElapsed && currentCommandSettledTime.milliseconds() > SETTLE_TIME_MILLIS) {
            Log.d("evaluateDrivingAutonomously", "Command completed, popping command");
            drive.setWeightedDrivePower(new Pose2d());
            commandSequence.remove(0);
            currentCommand = null;
        }

        return NibusState.AUTONOMOUSLY_DRIVING;
    }

    private NibusState evaluateDetectPoseFromAprilTag() {

        AprilTagDetection approachDetection = null;
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        for (AprilTagDetection detection : detections) {
            // Priority to some tags for positional approach...consider removing to focus on scoring approach use case.
            if (detection.id == 2 ||
                    detection.id == 5 ||
                    detection.id == 7 ||
                    detection.id == 10) {
                approachDetection = detection;
                break;
            }
        }

        if (approachDetection != null) {

            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            double rangeError = (approachDetection.ftcPose.range - DESIRED_DISTANCE);
            double headingError = Angle.normDelta(Math.toRadians(approachDetection.ftcPose.bearing));
            double yawError = Angle.normDelta(Math.toRadians(approachDetection.ftcPose.yaw));

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            double speed = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            double turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            double strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            telemetry.addData("Auto", "Speed %5.2f, Strafe %5.2f, Turn %5.2f ", speed, strafe, turn);

            if (Math.abs(rangeError) < DISTANCE_ERROR_THRESHOLD &&
                    Math.abs(headingError) < TURN_ERROR_THRESHOLD &&
                    Math.abs(yawError) < STRAFE_ERROR_THRESHOLD) {
                if (approachSettlingTimer == null) {
                    approachSettlingTimer = new ElapsedTime();
                    approachSettlingTimer.reset();
                }
            } else if (approachSettlingTimer != null) {
                approachSettlingTimer.reset();
            }

            drive.setWeightedDrivePower(new Pose2d(speed, strafe, turn));
        } else {
            if (approachSettlingTimer != null) {
                approachSettlingTimer.reset();
            }
            drive.setWeightedDrivePower(new Pose2d());
        }

        if (b1PressedEvaluator.evaluate()) {
            approachSettlingTimer = null;
            return NibusState.MANUAL_DRIVE;
        }

        if (approachSettlingTimer != null && approachSettlingTimer.milliseconds() > APPROACH_SETTLE_TIME_MS) {
            Pose2d settledPose = calculateRobotPoseWhenApproachToAprilTagSettled(approachDetection);
            setPoseEstimate(settledPose);
            lastAprilTagFieldPosition = settledPose;
            approachSettlingTimer = null;
            return NibusState.MANUAL_DRIVE;
        }

        return NibusState.DETECT_POSE_FROM_APRIL_TAG;
    }

    private Pose2d calculateRobotPoseWhenApproachToAprilTagSettled(AprilTagDetection detection) {
        CenterStageAprilTags tag = CenterStageAprilTags.getTag(detection.id);
        if (tag == null) return null;

        double tagHeading = tag.Pose.getHeading();
        // Assume we are squared to the april tag cause we settled
        double robotXFieldPosition = tag.Pose.getX() + ((DESIRED_DISTANCE + FRONT_CAMERA_OFFSET_INCHES) * Math.cos(tagHeading));
        double robotYFieldPosition = tag.Pose.getY();

        return new Pose2d(robotXFieldPosition, robotYFieldPosition, Angle.norm(tagHeading + Math.PI));
    }

    private Pose2d calculateRobotPose(AprilTagDetection detection, double cameraRobotOffset, double cameraRobotHeadingOffset) {
        CenterStageAprilTags tag = CenterStageAprilTags.getTag(detection.id);
        if (tag == null) return null;

        double yaw = Math.toRadians(detection.ftcPose.yaw);
        double bearing = Math.toRadians(detection.ftcPose.bearing);
        double range = detection.ftcPose.range;
        double tagToCameraHeading = Angle.norm(tag.Pose.getHeading() + bearing - yaw);
        double cameraFieldX = tag.Pose.getX() + (range * Math.cos(tagToCameraHeading));
        double cameraFieldY = tag.Pose.getY() + (range * Math.sin(tagToCameraHeading));
        double cameraFieldHeading = Angle.norm(
                tag.Pose.getHeading() + Math.PI + cameraRobotHeadingOffset - yaw);

        double robotFieldX = cameraFieldX - (cameraRobotOffset * Math.cos(cameraFieldHeading));
        double robotFieldY = cameraFieldY - (cameraRobotOffset * Math.sin(cameraFieldHeading));

        return new Pose2d(robotFieldX, robotFieldY, cameraFieldHeading);
    }

    private NibusState evaluateDetectAllianceMarker() {
        if (elapsedPropFrameTime.milliseconds() > FRAME_DELAY_MILLIS && framesProcessed < (DELAY_FRAMES + PROCESS_FRAMES)) {
            elapsedPropFrameTime.reset();
            Object[] results = propFinder.topbox(PROP_CAMERA_WIDTH_PIXELS, PROP_CAMERA_HEIGHT_PIXELS, PROP_CAMERA_ROW_COUNT, PROP_CAMERA_COLUMN_COUNT, allianceColor);
            if (results.length > 0) {
                if (framesProcessed >= DELAY_FRAMES) {
                    int voteIndex = (int) results[1];
                    leftMidRightVotes[voteIndex]++;
                    Log.d("nibusVision", String.format("Vision vote for %d", voteIndex));
                } else {
                    Log.d("nibusVision", "Delaying this frame.");
                }
                framesProcessed++;
            } else {
                Log.d("nibusVision", "No frame processed");
            }
        }

        if (framesProcessed >= DELAY_FRAMES + PROCESS_FRAMES) {
            int leftVotes = leftMidRightVotes[0];
            int midVotes = leftMidRightVotes[1];
            int rightVotes = leftMidRightVotes[2];
            if (leftVotes >= midVotes && leftVotes >= rightVotes) {
                alliancePropPosition = AlliancePropPosition.LEFT;
            } else if (midVotes > leftVotes && midVotes >= rightVotes) {
                alliancePropPosition = AlliancePropPosition.MID;
            } else {
                alliancePropPosition = AlliancePropPosition.RIGHT;
            }

            Log.d("evaluateDetectAllianceMarker", String.format("Detected %s", alliancePropPosition.name()));

            if (autonomousPlan != null) {
                setAutonomousCommands(
                        NibusState.STOPPED_UNTIL_END,
                        autonomousPlan.autonomousCommandsAfterPropDetect(allianceColor, alliancePropPosition));
                return NibusState.AUTONOMOUSLY_DRIVING;
            } else {
                return NibusState.MANUAL_DRIVE;
            }
        }

        return NibusState.DETECT_ALLIANCE_MARKER;
    }

    private Trajectory buildAutonomousTrajectoryFromHere(Vector2d vector2d, double heading) {
        return buildAutonomousTrajectoryFromHere(convertToPose(vector2d, heading));
    }

    private Trajectory buildAutonomousTrajectoryFromHere(Pose2d pose2d) {
        return drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(pose2d,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * .66, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
    }

    private void setAutonomousCommands(NibusState _continuationState, List<NibusCommand> commands) {
        commandSequence.clear();
        commandSequence.addAll(commands);
        continuationState = _continuationState;
    }

    private Pose2d getScaledHeadlessDriverInput(Gamepad gamepad) {
        Vector2d inputFieldDirection = NibusHelpers.headlessLeftStickFieldDirection(gamepad, allianceColor.OperatorHeadingOffset, latestPoseEstimate.getHeading());
        double scale = gamepad.right_trigger > 0.2 ? SLOW_MODE_SCALE : FAST_MODE_SCALE;
        double scaledRobotX = inputFieldDirection.getX() * scale;
        double scaledRobotY = inputFieldDirection.getY() * scale;
        double scaledRotation = -gamepad.right_stick_x * scale;
        return new Pose2d(scaledRobotX, scaledRobotY, scaledRotation);
    }

    private Pose2d getRobotHeadedFinesseInput(Gamepad gamepad, Pose2d fixedPose) {
        final double FORWARD_ALLOWANCE = 18;
        final double LATERAL_ALLOWANCE = 6;
        final double THETA_ALLOWANCE = Math.PI / 4;

        double forwardHeading = isUpstage() ?
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

    private void controlDrivingFromGamepad() {

    }

    private double armTargetPosition(CollectorState collectorState) {
        Integer armNudgeCountBoxed = armNudgesPerPosition.getOrDefault(collectorState, 0);
        int armNudgeCount = armNudgeCountBoxed == null ? 0 : armNudgeCountBoxed;
        double trimmedArmTargetDegrees = collectorState.ArmPosition + (armNudgeCount * ARM_DEGREE_TRIM_INCREMENT);
        return (((Math.min(trimmedArmTargetDegrees, ARM_MAX_ANGLE)) - ARM_DEGREE_OFFSET_FROM_HORIZONTAL) * ARM_TICKS_PER_DEGREE);
    }

    private Mat.Tuple3<Double> armExtenderWristPositionsForScoringHeight(double scoringHeightInches) {

        double ARM_MINIMUM_LENGTH_INCHES = 18;
        double BACKDROP_ANGLE = Math.PI / 3;
        int HORIZONTAL_TICKS = 5818;
        double WRIST_LOW_SETPOINT = .63;
        double WRIST_HIGH_SETPOINT = .39;
        double WRIST_RANGE = WRIST_HIGH_SETPOINT - WRIST_LOW_SETPOINT;

        double xBackdrop = scoringHeightInches / Math.tan(BACKDROP_ANGLE);
        double x = ARM_MINIMUM_LENGTH_INCHES + xBackdrop;
        double y = scoringHeightInches;
        double armAngle = Math.atan2(y, x);
        double length = Math.sqrt((x * x) + (y * y));
        double extension = Math.min(length - ARM_MINIMUM_LENGTH_INCHES, EXTENDER_MAX_LENGTH_INCHES);
        double extensionPercentage = extension / EXTENDER_MAX_LENGTH_INCHES;

        double armTargetTicks = HORIZONTAL_TICKS - (Math.toDegrees(armAngle) * ARM_TICKS_PER_DEGREE);
        double extenderTargetTicks = extension * EXTENDER_TICS_PER_INCH;
        double wristTargetPosition = WRIST_LOW_SETPOINT; //+ (extensionPercentage * WRIST_RANGE);

        return new Mat.Tuple3<>(armTargetTicks, extenderTargetTicks, wristTargetPosition);
    }

    private void controlArmMotor(DcMotorEx armMotor, int targetPosition) {
        int armPosition = armMotor.getCurrentPosition();
        double armPower = armcontrol.calculate(armPosition, targetPosition);
        if(Math.abs(armPower) < 0.02) armPower = 0;
        if(armPower < 0 && armMin.isPressed()) armPower = 0;
        telemetry.addData("armMotorPower", "%s: %5.2f", armMotor.getDeviceName(), armPower);
        telemetry.addData("armPosition", "%s: %d", armMotor.getDeviceName(), armPosition);
        armMotor.setPower(armPower);
    }

    private void setManualArmState() {
        double scoringHeightInches = scoringHeightOffset;

        Mat.Tuple3<Double> targets = armExtenderWristPositionsForScoringHeight(Math.max(0, scoringHeightInches));
        armTickTarget = targets.get_0();
        extenderTickTarget = targets.get_1();
        wristPosition = targets.get_2();
        this.collectorState = null;
    }

    private void setCollectorState(CollectorState collectorState) {
        this.collectorState = collectorState;

        Integer extenderNudges = extenderNudgesPerPosition.getOrDefault(collectorState, 0);
        int extenderNudgesCount = extenderNudges == null ? 0 : extenderNudges;
        Integer wristNudgeCountBoxed = wristNudgesPerPosition.getOrDefault(collectorState, 0);
        int wristNudgeCount = wristNudgeCountBoxed == null ? 0 : wristNudgeCountBoxed;

        double effectiveExtenderPositionInches = Math.max(0f, Math.min(collectorState.ExtenderPosition + (extenderNudgesCount/2.54f), EXTENDER_MAX_LENGTH_INCHES));
        armTickTarget = armTargetPosition(collectorState);
        extenderTickTarget = effectiveExtenderPositionInches * EXTENDER_TICS_PER_INCH;
        wristPosition = Math.min(1, Math.max(0, collectorState.WristPosition + (wristNudgeCount * WRIST_SERVO_TRIM_INCREMENT)));
    }

    private void controlScoringSystems() {
        controlArmMotor(armLeft, (int) armTickTarget);
        controlArmMotor(armRight, (int) armTickTarget);

        telemetry.addData("Extender position: ", extender.getCurrentPosition());
        telemetry.addData("Extender target position: ", extenderTickTarget);
        extender.setTargetPosition((int) extenderTickTarget);

        telemetry.addData("Wrist position: ", wristPosition);
        wrist.setPosition(wristPosition);

        greenGrabber.setPosition(greenGrabberState.ServoPosition + greenGrabberManualOffset);
        blueGrabber.setPosition(blueGrabberState.ServoPosition + blueGrabberManualOffset);

        if (launchingAirplane) {
            int launchSequenceTimeMillis = (int) timeSinceStart.milliseconds() - launchingAirplaneTimeMillis;
            if (launchSequenceTimeMillis > 1000) {
                launcher.setPower(1);
            }

            if (launchSequenceTimeMillis > 2000) {
                launcher.setPower(0);
                launchingAirplane = false;
            }
        }
    }

    private boolean armAndExtenderSettled() {
        int leftArmErrorTicks = (int) Math.abs(armTickTarget - armLeft.getCurrentPosition());
        int rightArmArmErrorTicks = (int) Math.abs(armTickTarget - armRight.getCurrentPosition());
        int extenderErrorTicks = Math.abs(extender.getTargetPosition() - extender.getCurrentPosition());
        return leftArmErrorTicks < ARM_TICK_TOLERANCE &&
                rightArmArmErrorTicks < ARM_TICK_TOLERANCE &&
                extenderErrorTicks < EXTENDER_TICK_TOLERANCE;
    }

    private void grabberInit() {
        // Initial short cycle to visually indicate ready to load pixels
        blueGrabberState = BlueGrabberState.NOT_GRABBED;
        greenGrabberState = GreenGrabberState.NOT_GRABBED;
        greenGrabber.setPosition(greenGrabberState.ServoPosition);
        blueGrabber.setPosition(blueGrabberState.ServoPosition);

        sleep(500);

        blueGrabberState = BlueGrabberState.GRABBED;
        greenGrabberState = GreenGrabberState.GRABBED;
        greenGrabber.setPosition(greenGrabberState.ServoPosition);
        blueGrabber.setPosition(blueGrabberState.ServoPosition);

        sleep(500);

        // Then hold open for 3 seconds before closing
        blueGrabberState = BlueGrabberState.NOT_GRABBED;
        greenGrabberState = GreenGrabberState.NOT_GRABBED;
        greenGrabber.setPosition(greenGrabberState.ServoPosition);
        blueGrabber.setPosition(blueGrabberState.ServoPosition);

        sleep(3000);

        blueGrabberState = BlueGrabberState.GRABBED;
        greenGrabberState = GreenGrabberState.GRABBED;
        greenGrabber.setPosition(greenGrabberState.ServoPosition);
        blueGrabber.setPosition(blueGrabberState.ServoPosition);
    }

    private void extenderInitSequence(boolean slow) {
        zeroExtender();

        extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extender.setDirection(DcMotorEx.Direction.FORWARD);
        extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        while (!extenderMin.isPressed()) {
            extender.setPower(-0.4);
        }

        extender.setPower(0);

        if (slow) {
            while (extenderMin.isPressed()) {
                extender.setPower(0.4);
            }

            sleep(500);

            while (!extenderMin.isPressed()) {
                extender.setPower(-0.1);
            }

            extender.setPower(0);
        }

        zeroExtender();
    }


    private void armInitSequence(boolean slow) {
        zeroArmMotors();

        while (!armMin.isPressed()){
            armLeft.setPower(-0.30);
            armRight.setPower(-0.30);
        }

        if (slow) {
            while (armMin.isPressed()){
                armLeft.setPower(0.40);
                armRight.setPower(0.40);
            }

            sleep(500);

            while (!armMin.isPressed()) {
                armLeft.setPower(-0.10);
                armRight.setPower(-0.10);
            }
        }

        zeroArmMotors();
    }

    public Pose2d convertToPose(Vector2d vector2d, double heading) {
        return new Pose2d(vector2d.getX(), vector2d.getY(), heading);
    }

    private boolean isEndgame() {
        return timeSinceStart.milliseconds() > END_GAME_BEGINS_MILLIS;
    }

    private boolean hasPositionEstimate() {
        return lastAprilTagFieldPosition != null && latestPoseEstimate != null;
    }

    private boolean isUpstage() {
        return hasPositionEstimate() && latestPoseEstimate.getX() < CenterStageConstants.UPSTAGE_FIELD_X;
    }

    private boolean isBackstage() {
        return hasPositionEstimate() && latestPoseEstimate.getX() > CenterStageConstants.BACKSTAGE_FIELD_X;
    }

    private boolean isInterstage() {
        return hasPositionEstimate() && !isUpstage() && !isBackstage();
    }

    public void shutDown() {
        drive.setWeightedDrivePower(new Pose2d());
        visionPortal.close();
    }
}
