package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Processors.WindowBoxesVisionProcessor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.AlliancePose;
import org.firstinspires.ftc.teamcode.util.AlliancePropPosition;
import org.firstinspires.ftc.teamcode.util.BlueGrabberState;
import org.firstinspires.ftc.teamcode.util.CollectorState;
import org.firstinspires.ftc.teamcode.util.GreenGrabberState;
import org.firstinspires.ftc.teamcode.util.OnActivatedEvaluator;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.ArrayList;
import java.util.Arrays;

public class Nibus2000 {

    public static final int ARM_TOLERANCE = 10;

    private static final double GEAR_RATIO = 13.7d;

    private static final double WORM_RATIO = 28.0d;

    private static final double ARM_TICKS_PER_DEGREE = WORM_RATIO * 28.0d * GEAR_RATIO / 360.0d;

    // Angle below horiontal at start in degrees.  horizontal is 0.
    public static final double ARM_DEGREE_OFFSET_FROM_HORIZONTAL = -37d;

    public static final double ARM_MAX_ANGLE = 180d;

    public static final double EXTENDER_MAX_LENGTH = 19d;

    public static final double EXTENDER_POWER = 0.8d;

    public static final double EXTENDER_GEAR_RATIO = 5.2d;

    public static final double EXTENDER_TICS_PER_CM = EXTENDER_GEAR_RATIO * 28 / 0.8;

    public static int PROP_CAMERA_WIDTH_PIXELS = 640;
    public static int PROP_CAMERA_HEIGHT_PIXELS = 480;
    public static int PROP_CAMERA_ROW_COUNT = 3;
    public static int PROP_CAMERA_COLUMN_COUNT = 3;

    private SampleMecanumDrive drive;
    private TouchSensor armMin;
    private TouchSensor extenderMin;
    private PIDController armcontrol;

    public static double p = 0.005, i = 0.005, d = 0.0002;

    //Length to extend in cm
    public static double extendLength = 0;

    public static int extendTicTarget = 0;

    public static double armTargetDegrees = 0;
    public static double target = 0.0;


    private DcMotorEx arm_motor0;

    private DcMotorEx extender;

    private DcMotorEx launcher;

    //collection servos
    Servo greenGrabber;

    Servo blueGrabber;

    Servo wrist;
    Servo launcherWrist;
    private CollectorState collectorState = CollectorState.DRIVING_SAFE;

    private OnActivatedEvaluator a1PressedEvaluator;
    private OnActivatedEvaluator y1PressedEvaluator;
    private OnActivatedEvaluator b1PressedEvaluator;
    private OnActivatedEvaluator lb1PressedEvaluator;
    private OnActivatedEvaluator rb1PressedEvaluator;
    private OnActivatedEvaluator a2PressedEvaluator;
    private OnActivatedEvaluator b2PressedEvaluator;
    private OnActivatedEvaluator x2PressedEvaluator;
    private OnActivatedEvaluator y2PressedEvaluator;
    private OnActivatedEvaluator lb2PressedEvaluator;
    private OnActivatedEvaluator rb2PressedEvaluator;
    private OnActivatedEvaluator driveNotBusyEvaluator;
    private BlueGrabberState blueGrabberState = BlueGrabberState.NOT_GRABBED;
    private GreenGrabberState greenGrabberState = GreenGrabberState.NOT_GRABBED;

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;

    private AllianceColor allianceColor;
    private AlliancePose alliancePose;
    private NibusState state;

    private ElapsedTime timeSinceStart;
    private ElapsedTime timeInState;
    private VisionPortal visionPortal;
    private WindowBoxesVisionProcessor propFinder;
    private Pose2d latestPoseEstimate = null;
    private int armStartingTickOffset = 0;
    private int extenderStartingTickOffset = 0;

    private int currentArmPosition = 0;
    private int currentExtenderPosition = 0;

    public Nibus2000(AllianceColor allianceColor, Gamepad gamepad1, Gamepad gamepad2, HardwareMap hardwareMap, Telemetry telemetry) {
        this.allianceColor = allianceColor;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.state = NibusState.MANUAL_DRIVE;

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        driveNotBusyEvaluator = new OnActivatedEvaluator(() -> !drive.isBusy());
        a1PressedEvaluator = new OnActivatedEvaluator(() -> gamepad1.a);
        b1PressedEvaluator = new OnActivatedEvaluator(() -> gamepad1.b);
        y1PressedEvaluator = new OnActivatedEvaluator(() -> gamepad1.y);
        lb1PressedEvaluator = new OnActivatedEvaluator(() -> gamepad1.left_bumper);
        rb1PressedEvaluator = new OnActivatedEvaluator(() -> gamepad1.right_bumper);
        a2PressedEvaluator = new OnActivatedEvaluator(() -> gamepad2.a);
        b2PressedEvaluator = new OnActivatedEvaluator(() -> gamepad2.b);
        x2PressedEvaluator = new OnActivatedEvaluator(() -> gamepad2.x);
        y2PressedEvaluator = new OnActivatedEvaluator(() -> gamepad2.y);
        lb2PressedEvaluator = new OnActivatedEvaluator(() -> gamepad2.left_bumper);
        rb2PressedEvaluator = new OnActivatedEvaluator(() -> gamepad2.right_bumper);

        arm_motor0 = hardwareMap.get(DcMotorEx.class, "arm_motorE0");
        extender = hardwareMap.get(DcMotorEx.class, "extenderE1");
        greenGrabber = hardwareMap.get(Servo.class, "greenE0");
        blueGrabber = hardwareMap.get(Servo.class, "blueE1");
        wrist = hardwareMap.get(Servo.class, "redE3");
        launcher = hardwareMap.get(DcMotorEx.class, "launcherE2");
        launcherWrist = hardwareMap.get(Servo.class, "launcher");
        armMin = hardwareMap.get(TouchSensor.class, "armMin1");
        extenderMin = hardwareMap.get(TouchSensor.class, "extenderMin3");
        armcontrol = new PIDController(p, i, d);
    }

    public void autonomousInit(AlliancePose alliancePose) {
        this.alliancePose = alliancePose;
        drive.setPoseEstimate((allianceColor.getAbsoluteFieldPose(alliancePose)));

        wrist.setPosition(.8);
        sleep(1000);
        autoHomeCollectorLoop();
        grabberInit();
        wrist.setPosition(1);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    public void teleopInit(NibusSaveState saveState) {
        arm_motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_motor0.setDirection(DcMotorSimple.Direction.FORWARD);
        arm_motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extender.setTargetPosition(0);
        extender.setTargetPositionTolerance(ARM_TOLERANCE);
        extender.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        if (saveState != null) {
            drive.setPoseEstimate(saveState.Pose);
            armStartingTickOffset = saveState.ArmPosition;
            extenderStartingTickOffset = saveState.ExtenderPosition;
            Log.d("Nibus2000", String.format("Setting arm offset ticks: %d", armStartingTickOffset));
            Log.d("Nibus2000", String.format("Setting extender offset ticks: %d", extenderStartingTickOffset));
            blueGrabberState = saveState.BlueGrabberState;
            greenGrabberState = saveState.GreenGrabberState;
            applyCollectorState(saveState.CollectorState);
        }
    }

    public void startup(NibusState startupState) {
        timeSinceStart = new ElapsedTime();
        timeInState = new ElapsedTime();
        state = startupState;
    }

    public boolean evaluate() {
        // Update drive on every cycle to keep the odometry position in sync at all times.
        drive.update();
        // Control arm and grabber every cycle
        applyCollectorState(collectorState);
        controlArmAndExtender();

        greenGrabber.setPosition(greenGrabberState.ServoPosition);
        blueGrabber.setPosition(blueGrabberState.ServoPosition);

        NibusState currentState = state;
        NibusState nextState = state;
        latestPoseEstimate = drive.getPoseEstimate();
        switch (state) {
            case MANUAL_DRIVE:
                nextState = evaluateDrivingAndScoring();
                break;
            case AUTONOMOUSLY_DRIVING:
                nextState = evaluateDrivingAutonomously();
                break;
            case DETECT_ALLIANCE_MARKER:
                nextState = evaluateDetectAllianceMarker();
                break;
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

    private NibusState evaluateDrivingAndScoring() {
        controlDrivingFromGamepad();
        evaluateScoringManualControls();
        if (gamepad1.y){
            planeLaunch();
        }
        if (b1PressedEvaluator.evaluate()) {
            stop();
            return NibusState.HALT_OPMODE;
        }
        return NibusState.MANUAL_DRIVE;
    }
    private void planeLaunch() {
        launcherWrist.setPosition(.5);
        sleep(1000);
        launcher.setPower(1);
    }

    private ElapsedTime currentCommandTime = new ElapsedTime();
    private NibusAutonomousCommand currentCommand = null;

    private ArrayList<NibusAutonomousCommand> commandSequence = new ArrayList<NibusAutonomousCommand>();
    private NibusState continuationState = null;

    private NibusState evaluateDrivingAutonomously() {
        if (commandSequence.size() == 0) {
            NibusState _continuationState = continuationState;
            continuationState = null;
            return _continuationState == null ? NibusState.MANUAL_DRIVE : _continuationState;
        }

        if (currentCommand == null) {
            currentCommandTime.reset();
            currentCommand = commandSequence.get(0);

            if (currentCommand.BlueGrabberState != null) {
                blueGrabberState = currentCommand.BlueGrabberState;
            }

            if (currentCommand.GreenGrabberState != null) {
                greenGrabberState = currentCommand.GreenGrabberState;
            }

            if (currentCommand.CollectorState != null) {
                Log.d("evaluateDrivingAutonomously", "Setting collector state");
                applyCollectorState(currentCommand.CollectorState);
            }

            if (currentCommand.TrajectoryCreator != null) {
                Log.d("evaluateDrivingAutonomously", "Following trajectory async");
                drive.followTrajectoryAsync(currentCommand.TrajectoryCreator.create());
            }
        }

        boolean collectorSettled = currentCommand.CollectorState == null || armAndExtenderSettled();
        boolean driveCompleted = currentCommand.TrajectoryCreator == null || !drive.isBusy();
        boolean minTimeElapsed = currentCommandTime.milliseconds() > currentCommand.MinTimeMillis;
        boolean commandComplete = minTimeElapsed && collectorSettled && driveCompleted;
        if (commandComplete) {
            Log.d("evaluateDrivingAutonomously", "Command completed, popping command");
            commandSequence.remove(0);
            currentCommand = null;
        }

        return NibusState.AUTONOMOUSLY_DRIVING;
    }

    private static int FRAME_DELAY_MILLIS = 100;
    private static int DELAY_FRAMES = 10;
    private static int PROCESS_FRAMES = 10;
    private int framesProcessed = 0;
    private int[] leftMidRightVotes = new int[] { 0, 0, 0 };
    private AlliancePropPosition alliancePropPosition = null;
    private ElapsedTime elapsedPropFrameTime = new ElapsedTime();

    private void prepareToDetectAllianceMarker() {
        framesProcessed = 0;
        leftMidRightVotes[0] = 0;
        leftMidRightVotes[1] = 0;
        leftMidRightVotes[2] = 0;
        alliancePropPosition = null;
        elapsedPropFrameTime.reset();
    }

    private NibusState evaluateDetectAllianceMarker() {

        // Lazy init the prop finder and vision portal
        if (propFinder == null) {
            propFinder = new WindowBoxesVisionProcessor();
        }

        if (visionPortal == null) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 2"), propFinder);
        }

        // Idempotent
        visionPortal.resumeStreaming();

        if (elapsedPropFrameTime.milliseconds() > FRAME_DELAY_MILLIS && framesProcessed < (DELAY_FRAMES + PROCESS_FRAMES)) {
            elapsedPropFrameTime.reset();
            Object[] redResults = propFinder.topbox(PROP_CAMERA_WIDTH_PIXELS, PROP_CAMERA_HEIGHT_PIXELS, PROP_CAMERA_ROW_COUNT, PROP_CAMERA_COLUMN_COUNT, allianceColor);
            if (redResults.length > 0) {
                if (framesProcessed >= DELAY_FRAMES) {
                    int voteIndex = (int) redResults[1];
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
            deinitPropFider();
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

            if (alliancePose != null) {
                Vector2d targetLocation = alliancePose.getPixelTargetPosition(allianceColor, alliancePropPosition);
                double targetHeading = Math.toRadians(90);
                switch (allianceColor) {
                    case RED:
                        targetHeading += Math.toRadians(45);
                    case BLUE:
                        targetHeading -= Math.toRadians(45);
                    default:
                        break;
                }

                Pose2d targetPose = new Pose2d(targetLocation.getX(), targetLocation.getY(), targetHeading);
                Pose2d approachPose = calculateApproachPose(targetPose, -8);

                Vector2d waypoint1 = allianceColor.getMiddleLaneAudienceWaypoint();
                Vector2d waypoint2 = allianceColor.getMiddleLaneBackstageWaypoint();
                Vector2d waypoint3 = allianceColor.getScoringApproachLocation();

                setAutonomousCommands(NibusState.MANUAL_DRIVE,
                        new NibusAutonomousCommand(CollectorState.CLOSE_COLLECTION),
                        new NibusAutonomousCommand(
                                () -> drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineToSplineHeading(approachPose)
                                    .build()),
                        new NibusAutonomousCommand(BlueGrabberState.NOT_GRABBED, GreenGrabberState.GRABBED),
                        new NibusAutonomousCommand(CollectorState.DRIVING_SAFE),
                        new NibusAutonomousCommand(
                                () -> drive.trajectoryBuilder(drive.getPoseEstimate())
                                        .lineToSplineHeading(convertToPose(waypoint1, 0))
                                        .build()),
                        new NibusAutonomousCommand(
                                () -> drive.trajectoryBuilder(drive.getPoseEstimate())
                                        .lineToSplineHeading(convertToPose(waypoint2, 0))
                                        .build()),
                        new NibusAutonomousCommand(
                                () -> drive.trajectoryBuilder(drive.getPoseEstimate())
                                        .lineToSplineHeading(convertToPose(waypoint3, 0))
                                        .build()),
                        new NibusAutonomousCommand(CollectorState.HIGH_SCORING),
                        new NibusAutonomousCommand(BlueGrabberState.NOT_GRABBED, GreenGrabberState.NOT_GRABBED),
                        new NibusAutonomousCommand(CollectorState.DRIVING_SAFE));

                return NibusState.AUTONOMOUSLY_DRIVING;
            } else {
                return NibusState.MANUAL_DRIVE;
            }
        }

        return NibusState.DETECT_ALLIANCE_MARKER;
    }

    private void setAutonomousCommands(NibusState _continuationState, NibusAutonomousCommand ... commands) {
        commandSequence.clear();
        commandSequence.addAll(Arrays.asList(commands));
        continuationState = _continuationState;
    }

    private void deinitPropFider() {
        visionPortal.stopStreaming();
    }

    private void controlDrivingFromGamepad() {
        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x));
    }

    private void getReadyToMove() {
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drive.update();
    }

    private void stop() {
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.update();
    }

    private void controlArmAndExtender() {
        target = computeArmTickTarget(armTargetDegrees);
        armcontrol.setPID(p, i, d);
        armcontrol.setTolerance(ARM_TOLERANCE);

        currentArmPosition = arm_motor0.getCurrentPosition();
        double armpower = armcontrol.calculate(currentArmPosition, target);

        //Set extension
        extendTicTarget = (int) computeExtenderTickTarget(extendLength);
        currentExtenderPosition = extender.getCurrentPosition();
        extender.setTargetPosition(extendTicTarget);

        if(armpower < 0 && armMin.isPressed()) armpower = 0;
        arm_motor0.setPower(armpower);
        extender.setPower(EXTENDER_POWER);

        telemetry.addData("Armpower", armpower);
    }

    private double computeArmTickTarget(double degreesOffsetFromHorizontal) {
        return (((Math.min(degreesOffsetFromHorizontal, ARM_MAX_ANGLE)) - ARM_DEGREE_OFFSET_FROM_HORIZONTAL) * ARM_TICKS_PER_DEGREE) - armStartingTickOffset;
    }

    private double computeExtenderTickTarget(double extendLength) {
        return ((Math.min(extendLength, EXTENDER_MAX_LENGTH)) * EXTENDER_TICS_PER_CM) - extenderStartingTickOffset;
    }

    private boolean armAndExtenderSettled() {
        int armErrorTicks = Math.abs((int) target - arm_motor0.getCurrentPosition());
        int extenderErrorTicks = Math.abs((int) extendTicTarget - extender.getCurrentPosition());
        return armErrorTicks < 20 && extenderErrorTicks < 20;
    }

    private void evaluateScoringManualControls() {
        if (a1PressedEvaluator.evaluate()) {
            blueGrabberState = blueGrabberState.toggle();
            greenGrabberState = greenGrabberState.toggle();
        }

        if (lb1PressedEvaluator.evaluate()) {
            greenGrabberState = greenGrabberState.toggle();
        }

        if (rb1PressedEvaluator.evaluate()) {
            blueGrabberState = blueGrabberState.toggle();
        }

        if (a2PressedEvaluator.evaluate()) {
            collectorState = CollectorState.CLOSE_COLLECTION;
        } else if (b2PressedEvaluator.evaluate()) {
            collectorState = CollectorState.FAR_COLLECTION;
        } else if (x2PressedEvaluator.evaluate()) {
            collectorState = CollectorState.DRIVING_SAFE;
        } else if (y2PressedEvaluator.evaluate()) {
            collectorState = CollectorState.LOW_SCORING;
        } else if (rb2PressedEvaluator.evaluate()) {
            collectorState = CollectorState.HIGH_SCORING;
        } else if (lb2PressedEvaluator.evaluate()) {
            collectorState = CollectorState.DRIVING_SAFE;
        }
        telemetry.addData("Collector state", collectorState);
        telemetry.addData("Wrist pos:",wrist.getPosition());
    }

    private void applyCollectorState(CollectorState collectorState) {
        wrist.setPosition(collectorState.WristPosition);
        extendLength = collectorState.ExtenderPosition;
        armTargetDegrees = collectorState.ArmPosition;
    }



    private void evaluateGrabber() {
        if (a1PressedEvaluator.evaluate()) {
            blueGrabberState = blueGrabberState.toggle();
            greenGrabberState = greenGrabberState.toggle();
        }

        if (lb1PressedEvaluator.evaluate()) {
            greenGrabberState = greenGrabberState.toggle();
        }

        if (rb1PressedEvaluator.evaluate()) {
            blueGrabberState = blueGrabberState.toggle();
        }
    }

    private void grabberInit() {
        blueGrabberState = BlueGrabberState.NOT_GRABBED;
        greenGrabberState = GreenGrabberState.NOT_GRABBED;
        greenGrabber.setPosition(greenGrabberState.ServoPosition);
        blueGrabber.setPosition(blueGrabberState.ServoPosition);
        sleep(2500);
        blueGrabberState = BlueGrabberState.GRABBED;
        greenGrabberState = GreenGrabberState.GRABBED;
        greenGrabber.setPosition(greenGrabberState.ServoPosition);
        blueGrabber.setPosition(blueGrabberState.ServoPosition);
    }

    private void runTelemetry() {
        telemetry.addData("x", latestPoseEstimate.getX());
        telemetry.addData("y", latestPoseEstimate.getY());
        telemetry.addData("heading", latestPoseEstimate.getHeading());
//        telemetry.addData("f", f);
        //telemetry.addData("ff", ff);
        //telemetry.addData("Position", armPos);
//        telemetry.addData("propPosition", alliancePropPosition);
//        telemetry.addData("target", target);
        //telemetry.addData("AnglePos", (armPos / ARM_TICKS_PER_DEGREE) + ARM_DEGREE_OFFSET_FROM_HORIZONTAL);
//        telemetry.addData("angle target", armTargetDegrees);
//        telemetry.addData("Extender Tic Target", extendTicTarget);
        //telemetry.addData("Extender Current Tics", extendPos);
//        telemetry.addData("Extender Target Length cm", extendLength);
        //telemetry.addData("Extend Current Length", extendPos / EXTENDER_TICS_PER_CM);
//        telemetry.addData("extender power", extender.getPower());
//        telemetry.addData("green",greenGrabber.getPosition());
//        telemetry.addData("blue",blueGrabber.getPosition());
        telemetry.update();
    }

    private static void sleep(int millis) {
        try {
            Thread.sleep(millis);
        } catch (Exception e) {}
    }

    private void autoHomeCollectorLoop() {
        extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extender.setDirection(DcMotorEx.Direction.FORWARD);
        extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        while (!extenderMin.isPressed()) {
            extender.setPower(-0.4);
        }
        while (extenderMin.isPressed()) {
            extender.setPower(0.4);
        }
        sleep(500);

        while (!extenderMin.isPressed()) {
            extender.setPower(-0.1);
        }
        extender.setPower(0);
        extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extender.setTargetPosition(0);
        extender.setTargetPositionTolerance(ARM_TOLERANCE);
        extender.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        //home arm
        arm_motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_motor0.setDirection(DcMotorSimple.Direction.FORWARD);
        arm_motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (!armMin.isPressed()){
            arm_motor0.setPower(-0.65);
        }
        while (armMin.isPressed()){
            arm_motor0.setPower(0.65);
        }
        sleep(500);

        while (!armMin.isPressed()) {
            arm_motor0.setPower(-0.05);
        }
        arm_motor0.setPower(0);
        arm_motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public Pose2d getPoseEstimate() {
        return drive.getPoseEstimate();
    }

    public int getArmPosition() {
        return currentArmPosition;
    }

    public int getExtenderPosition() {
        return currentExtenderPosition;
    }

    public CollectorState getCollectorState() {
        return collectorState;
    }

    public BlueGrabberState getBlueGrabberState() {
        return blueGrabberState;
    }

    public GreenGrabberState getGreenGrabberState() {
        return greenGrabberState;
    }

    public Pose2d calculateApproachPose(Pose2d targetPose, double offsetDistance) {
        // Calculate offset components based on target heading
        double offsetX = offsetDistance * Math.cos(targetPose.getHeading());
        double offsetY = offsetDistance * Math.sin(targetPose.getHeading());

        // Calculate the robot's position by subtracting the offset from the target position
        double robotX = targetPose.getX() - offsetX;
        double robotY = targetPose.getY() - offsetY;

        // Return the robot's required pose
        return new Pose2d(robotX, robotY, targetPose.getHeading());
    }

    public Pose2d convertToPose(Vector2d vector2d, double heading) {
        return new Pose2d(vector2d.getX(), vector2d.getY(), heading);
    }
}
