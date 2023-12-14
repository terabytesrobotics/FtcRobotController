package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Processors.WindowBoxesVisionProcessor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.BlueGrabberState;
import org.firstinspires.ftc.teamcode.util.CollectorState;
import org.firstinspires.ftc.teamcode.util.GreenGrabberState;
import org.firstinspires.ftc.teamcode.util.OnActivatedEvaluator;
import org.firstinspires.ftc.vision.VisionPortal;

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

    public static int PROP_CAMERA_WIDTH_PIXELS = 480;
    public static int PROP_CAMERA_HEIGHT_PIXELS = 640;
    public static int PROP_CAMERA_ROW_COUNT = 2;
    public static int PROP_CAMERA_COLUMN_COUNT = 3;

    private SampleMecanumDrive drive;
    private TouchSensor armMin;
    private TouchSensor extenderMin;
    private PIDController armcontrol;

    public static double p = 0.005, i = 0.005, d = 0.0002;
    public static double f = 0.0;


    //Length to extend in cm
    public static double extendLength = 0;

    public static int extendTicTarget = 0;

    public double fmax = 0;

    public static double armTargetDegrees = 0;
    public static double target = 0.0;


    private DcMotorEx arm_motor0;

    private DcMotorEx extender;

    //collection servos
    Servo greenGrabber;

    Servo blueGrabber;

    Servo wrist;

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
    private BlueGrabberState blueGrabberState = BlueGrabberState.NOT_GRABBED;
    private GreenGrabberState greenGrabberState = GreenGrabberState.NOT_GRABBED;

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;

    private AllianceColor allianceColor;
    private NibusState state;

    private ElapsedTime timeSinceStart;
    private ElapsedTime timeInState;
    private VisionPortal visionPortal;
    private WindowBoxesVisionProcessor propFinder;

    public Nibus2000(AllianceColor allianceColor, Gamepad gamepad1, Gamepad gamepad2, HardwareMap hardwareMap, Telemetry telemetry) {
        this.allianceColor = allianceColor;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.state = NibusState.STOPPED;
    }

    public void init(Pose2d initialPose) {
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

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(initialPose);

        arm_motor0 = hardwareMap.get(DcMotorEx.class, "arm_motorE0");
        extender = hardwareMap.get(DcMotorEx.class, "extenderE1");
        greenGrabber = hardwareMap.get(Servo.class, "greenE0");
        blueGrabber = hardwareMap.get(Servo.class, "blueE1");
        wrist = hardwareMap.get(Servo.class, "redE3");

        armMin = hardwareMap.get(TouchSensor.class, "armMin1");
        extenderMin = hardwareMap.get(TouchSensor.class, "extenderMin3");
        armcontrol = new PIDController(p, i, d);

        // MOVEMENT HAPPENS HERE
        wrist.setPosition(.8);
        sleep(1000);
        autoHomeCollectorLoop();
        grabberInit();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    public void startup(NibusState startupState) {
        timeSinceStart = new ElapsedTime();
        timeInState = new ElapsedTime();
        state = startupState;
    }

    public void evaluate() {
        NibusState currentState = state;
        NibusState nextState = state;
        switch (state) {
            case STOPPED:
                nextState = evaluateStopped();
                break;
            case DRIVE_AND_SCORE:
                nextState = evaluateDrivingAndScoring();
                break;
            case DETECT_ALLIANCE_MARKER:
                nextState = evaluateStopped();
                break;
            default:
                break;
        }
        if (nextState != currentState) {
            timeInState.reset();
            state = nextState;
        }
        runTelemetry();
    }

    private NibusState evaluateStopped() {
        stop();
        if (b1PressedEvaluator.evaluate()) {
            getReadyToMove();
            return NibusState.DRIVE_AND_SCORE;
        }

        if (y1PressedEvaluator.evaluate()) {
            initPropFinder();
            return NibusState.DETECT_ALLIANCE_MARKER;
        }
        return NibusState.STOPPED;
    }

    private NibusState evaluateDrivingAndScoring() {
        controlScoringSystem();
        controlDrivingFromGamepad();
        if (b1PressedEvaluator.evaluate()) {
            stop();
            return NibusState.STOPPED;
        }
        return NibusState.DRIVE_AND_SCORE;
    }

    private NibusState evaluateDetectAllianceMarker() {
        if (y1PressedEvaluator.evaluate()) {
            deinitPropFider();
            return NibusState.STOPPED;
        }

        if (allianceColor == AllianceColor.RED) {
            Object[] redResults = propFinder.topbox(PROP_CAMERA_WIDTH_PIXELS, PROP_CAMERA_HEIGHT_PIXELS, PROP_CAMERA_ROW_COUNT, PROP_CAMERA_COLUMN_COUNT, "RED");
            telemetry.addLine("Red Row " + redResults[0]);
            telemetry.addLine("Red Col" + redResults[1]);
            telemetry.addLine("Red value " + redResults[2]);
        } else if (allianceColor == AllianceColor.BLUE) {
            Object[] blueResults = propFinder.topbox(PROP_CAMERA_WIDTH_PIXELS, PROP_CAMERA_HEIGHT_PIXELS, PROP_CAMERA_ROW_COUNT, PROP_CAMERA_COLUMN_COUNT, "BLUE");
            telemetry.addLine("Blue Row " + blueResults[0]);
            telemetry.addLine("Blue Col" + blueResults[1]);
            telemetry.addLine("Blue value " + blueResults[2]);
        }

        return NibusState.DETECT_ALLIANCE_MARKER;
    }

    private void initPropFinder() {
        propFinder = new WindowBoxesVisionProcessor();

        // Create the vision portal the easy way.
        //if (USE_WEBCAM) {
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), propFinder);
        visionPortal.resumeStreaming();
/*        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, propfinder);
        }*/
    }

    private void deinitPropFider() {
        visionPortal.stopStreaming();
        visionPortal.close();
        visionPortal = null;
        propFinder = null;
    }

    private void controlDrivingFromGamepad() {
        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x));
        drive.update();
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

    private void controlScoringSystem() {
        controlGrabber();

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

        wrist.setPosition(collectorState.WristPosition);
        extendLength = collectorState.ExtenderPosition;
        armTargetDegrees = collectorState.ArmPosition;

        target = ((Math.min(armTargetDegrees, ARM_MAX_ANGLE)) - ARM_DEGREE_OFFSET_FROM_HORIZONTAL) * ARM_TICKS_PER_DEGREE;
        armcontrol.setPID(p, i, d);
        armcontrol.setTolerance(ARM_TOLERANCE);

        int armPos = arm_motor0.getCurrentPosition();
        int extendPos = extender.getCurrentPosition();
        double pid = armcontrol.calculate(armPos, target);
        double ff = ((f) + (fmax * extendPos / (EXTENDER_MAX_LENGTH * EXTENDER_TICS_PER_CM))) * Math.cos(Math.toRadians(armTargetDegrees));
        double armpower = pid + ff;

        //Set extension
        extendTicTarget = (int) ((Math.min(extendLength, EXTENDER_MAX_LENGTH)) * EXTENDER_TICS_PER_CM);
        extender.setTargetPosition(extendTicTarget);

        if(armpower < 0 && armMin.isPressed()) armpower = 0;
        arm_motor0.setPower(armpower);
        extender.setPower(EXTENDER_POWER);

        telemetry.addData("Armpower", armpower);
    }

    private void controlGrabber() {
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

        greenGrabber.setPosition(greenGrabberState.ServoPosition);
        blueGrabber.setPosition(blueGrabberState.ServoPosition);
    }

    private void grabberInit() {
        blueGrabberState = BlueGrabberState.NOT_GRABBED;
        greenGrabberState = GreenGrabberState.NOT_GRABBED;
        controlGrabber();
        sleep(5000);
        blueGrabberState = BlueGrabberState.GRABBED;
        greenGrabberState = GreenGrabberState.GRABBED;
        controlGrabber();
    }

    private void runTelemetry() {
        telemetry.addData("f", f);
        //telemetry.addData("ff", ff);
        //telemetry.addData("Position", armPos);
        telemetry.addData("target", target);
        //telemetry.addData("AnglePos", (armPos / ARM_TICKS_PER_DEGREE) + ARM_DEGREE_OFFSET_FROM_HORIZONTAL);
        telemetry.addData("angle target", armTargetDegrees);
        telemetry.addData("Extender Tic Target", extendTicTarget);
        //telemetry.addData("Extender Current Tics", extendPos);
        telemetry.addData("Extender Target Length cm", extendLength);
        //telemetry.addData("Extend Current Length", extendPos / EXTENDER_TICS_PER_CM);
        telemetry.addData("extender power", extender.getPower());
        telemetry.addData("green",greenGrabber.getPosition());
        telemetry.addData("blue",blueGrabber.getPosition());
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
}
