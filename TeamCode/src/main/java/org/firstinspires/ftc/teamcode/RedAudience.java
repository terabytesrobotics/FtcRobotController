package org.firstinspires.ftc.teamcode;

import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.SHOT_RED;

import android.icu.text.CaseMap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.Processors.WindowBoxesVisionProcessor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Config
@Autonomous(group = "Competition")
public class RedAudience extends LinearOpMode {

  private WebcamName FrontCam = null;
 private WebcamName RearCam = null;

    /*
     * Motor Variables
     */
    private DcMotor leftFrontDrive = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive = null;  //  Used to control the right back drive wheel
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    //Arm section
    private PIDController armcontrol;
    public static double p = 0.005, i = 0.005, d = 0.0002;
    public static double f = 0.0;
    // Angle below horiontal at start in degrees.  horizontal is 0.
    public static double angleoffset = -25;
    //Max angle limit
    public static double maxangle = 135;

    public static double degtarget = 0;
    // arm target in tics
    public static double target = 0.0;
    public static double armTolerance = 10;
    //arm motor gear ratio
    private final double gear_ratio = 13.7;
    private static double worm_ratio = 28;
    private final double arm_ticks_per_degree = worm_ratio * 28 * gear_ratio / 360;
    private DcMotorEx arm_motor0;


    //Extender section
    private DcMotorEx extender;
    // Max arm extension in cm
    public static double maxextend = 20;
    public static int extendTolerance = 4;
    public static double extendPower = 0.8;
    //Length to extend in cm
    public static double extendLength = 0;
    public static int extendTicTarget = 0;
    public static double extendergearratio = (5.2);
    static double extender_tics_per_cm = extendergearratio * 28 / 0.8;

    // touch sensors
    private TouchSensor armMin;
    private TouchSensor extenderMin;




    //Servos
    Servo greenGrabber;
    Servo blueGrabber;
    Servo wrist;


    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    private static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    /**
     * The variable to store our instance of the prop finding vision processor.
     */private WindowBoxesVisionProcessor propfinder;
    public CameraCalibration calibration;

    /**
     * height and width of cam used for prop "Cam2"
     */
    int HeightCam2 = 480;
    int WidthCam2 = 640;

    /**
     * Rows and columns for prop detection grid
     */
    public static int rows = 3;
    public static int cols = 1;

    public static String color = "RED";//Change to BLUE as necessary.

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;               // Used to manage the video source.
    String propLocation;

    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern = SHOT_RED;//Change Pattern and color for Alliance and position;
    /**Blinkin Patterns using red, blue, or green
     * BLUE
     * RED
     * GREEN
     * BREATH_BLUE
     * BREATH_RED
     * DARK_BLUE
     * DARK_RED
     * DARK_GREEN
     * HEARTBEAT_BLUE
     * HEARTBEAT_RED
     * SHOT_BLUE
     * SHOT_RED
     * LIGHT_CHASE_BLUE
     * LIGHT_CHASE_RED
     * STROBE_BLUE
     * STROBE_RED
     *

    */

    private enum CollectorState {
        CLOSE_COLLECTION(-26, 0, 0.25f),
        FAR_COLLECTION(-20, 18, 0.8f),
        DRIVING_SAFE(0, 0, 0.5f),
        LOW_SCORING(170, 0, 0.8f),
        HIGH_SCORING(120, 18, 0.8f),
        SAFE_POSITION(0, 0, 0.5f);

        public float WristPosition;
        public int ArmPosition;

        public int ExtenderPosition;

        private CollectorState( int armPosition, int extenderPosition, float wristPosition) {
            this.ArmPosition = armPosition;
            this.ExtenderPosition = extenderPosition;
            this.WristPosition = wristPosition;
        }

    }
    @Override
    public void runOpMode() throws InterruptedException {

        leftFrontDrive = hardwareMap.get(DcMotor.class, "fl2");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "fr3");
        leftBackDrive = hardwareMap.get(DcMotor.class, "bl0");
        rightBackDrive = hardwareMap.get(DcMotor.class, "br1");
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        greenGrabber = hardwareMap.get(Servo.class, "greenE0");
        blueGrabber = hardwareMap.get(Servo.class, "blueE1");
        wrist = hardwareMap.get(Servo.class, "redE3");

        //arm and ex
        armMin = hardwareMap.get(TouchSensor.class, "armMin1");
        extenderMin = hardwareMap.get(TouchSensor.class, "extenderMin3");
        armcontrol = new PIDController(p, i, d);
        arm_motor0 = hardwareMap.get(DcMotorEx.class, "arm_motorE0");
        extender = hardwareMap.get(DcMotorEx.class, "extenderE1");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm_motor0.setDirection(DcMotorEx.Direction.FORWARD);
        extender.setDirection(DcMotorSimple.Direction.FORWARD);


        extender.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        extender.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        extender.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        //extender.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        extender.setTargetPosition(0);

        wrist.setPosition(0);

        wait(3000);

        greenGrabber.setPosition(0.3);
        blueGrabber.setPosition(.02);

        RearCam = hardwareMap.get(WebcamName.class, "Webcam 1");
        FrontCam = hardwareMap.get(WebcamName.class, "Webcam 2");
        //Sets up vision processor
        propfinder = new WindowBoxesVisionProcessor();
        // Create the vision portal the easy way.
        visionPortal = VisionPortal.easyCreateWithDefaults(
                FrontCam, propfinder);



        /**
         * Add Blinken code to id alliance and position
         */
        blinkinLedDriver.setPattern(pattern);

        // Wait for the DS start button to be touched.
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (color=="RED") {
            Object redresults[] = propfinder.topbox(WidthCam2, HeightCam2, rows, cols, "RED");
            if (redresults[1] instanceof Integer && (int) redresults[1] < 1 ) {
            propLocation = "left";
            }
            else if (redresults[1] instanceof Integer && (int) redresults[1] == 1) {
                propLocation = "center";
            }
            else if (redresults[1] instanceof Integer && (int) redresults[1] > 1) {
                propLocation = "right";
            }

           //Default = center
            else propLocation = "center";

            telemetry.addLine("Red Row " + redresults[0]);
            telemetry.addLine("Red Col" + redresults[1]);
            telemetry.addLine("Red value " + redresults[2]);


        }

        else if (color=="BLUE") {
            Object blueresults[] = propfinder.topbox(WidthCam2, HeightCam2, rows, cols, "BLUE");
            if (blueresults[1] instanceof Integer && (int) blueresults[1] < 1 ) {
                propLocation = "left";
            }
            else if (blueresults[1] instanceof Integer && (int) blueresults[1] == 1) {
                propLocation = "center";
            }
            else if (blueresults[1] instanceof Integer && (int) blueresults[1] > 1) {
                propLocation = "right";
            }
            else propLocation = "center";


            telemetry.addLine("Blue Row " + blueresults[0]);
            telemetry.addLine("Blue Col" + blueresults[1]);
            telemetry.addLine("Blue value " + blueresults[2]);

        }
        else return;
        /**Turn off front camera
         * Turn on rear camera and set up Apriltag processor
         */
        visionPortal.setActiveCamera(RearCam);
        visionPortal.setProcessorEnabled(propfinder,false);
        visionPortal.setProcessorEnabled(aprilTag,true);






        /**
         * Add drive to pose to drop first pixel based on propLocation left, center, right
         * Drop pixel
         * If on audience side, drive to backstage to score deployment position
         * Move arm and wrist to score position
         * Drive to April tag detect position
         * Do Apriltag alignment based on propLocation
         * Drop pixel
         */


        telemetry.addData("Prop position",propLocation);
        telemetry.update();


       /* SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        waitForStart();
        if (isStopRequested()) return;

        Trajectory traj = drive
                .trajectoryBuilder(new Pose2d())
                .build();
        drive.followTrajectory(traj);

        */
    }

    private void initpropfinder() {

        propfinder = new WindowBoxesVisionProcessor();

        // Create the vision portal the easy way.

            visionPortal = VisionPortal.easyCreateWithDefaults(
                    FrontCam, propfinder);
        }

       // end method initpropfinder()
}

