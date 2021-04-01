package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


@Autonomous(name="More Points Auto", group ="Real")


public class NewandImprovedAuto extends LinearOpMode {
    private static final int secondsPermissionTimeout = Integer.MAX_VALUE;
    private static final String TAG = "Webcam Sample";
    //  private Gyroscope imu;
    private RevColorSensorV3 REVColor1;
    private RevColorSensorV3 REVColor;


    private Timer timer;
    private ShooterCommands shooterCommands;
    private CLift CollectorMove;
    private DriveMotors driveMotors;
    private DcMotorEx Shooter;
    private DcMotor Collector;
    private DcMotor cLift;
    private Servo Platform;
    private Servo lLift;
    private Servo rLift;
    private Servo Trigger;
    private RevTouchSensor TopLimit;
    private RevTouchSensor BottomLimit;
    private Servo WobbleServo;
    private Servo LFinger;
    private Servo RFinger;
    private Servo Roller1;
    private Servo Roller;
    private Servo FinalRoller;

    // IMPORTANT: If you are using a USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AUSc2Kb/////AAABmdTkqFyT9UOrtDrK1zeFQvxTs4OQJgnhM43FLQWLfPnrpWlAkrKgsiMjrteGq85MoK1MKv/ugy48B51DZRiDIyr9ijqcNL8ZRiRgKQzrb064nctGU52JeYg1BKKqjogBv/yuh4qexWOvVjhwEDvYxVL7P/IXs1ERzSLTNoUQEMFV+BUoLD3vzU0z/7cQt9hj4IheiD1Di7Q/JX/0z5PMr5cBBMqeJqd1JxD6p6CzvkqM6KH3Gpok9mteCtSw6z65eQa2sLBPsiOxCUa9oM1IUZ9s1zlexvgO8q5EMkh4GJx/+jyvoVzuSvf9aC+HqUQvALsVcO2wwOvmlXbMuHg8dM4FUkp70zawiP0VHY8/TyJK";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    //private VuforiaTrackables targetsUltimateGoal = null;
    //private List<VuforiaTrackable> allTrackables = null;

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    WebcamName webcamName = null;

    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    /*
     * tfod is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private double PlatformPositionX;
    private boolean Left;
    private boolean Right;
    private double UpYes;

    @SuppressLint("DefaultLocale")
    @Override public void runOpMode() {
        //imu = hardwareMap.get(Gyroscope.class, "imu");
        LFinger = hardwareMap.get(Servo.class, "Finger");
        RFinger = hardwareMap.get(Servo.class, "Finger2");
        REVColor1 =hardwareMap.get(RevColorSensorV3.class,"REVColor1");
        REVColor = hardwareMap.get(RevColorSensorV3.class, "REVColor");

        driveMotors = new DriveMotors(
                hardwareMap.get(DcMotorEx.class, "flMotor"),
                hardwareMap.get(DcMotorEx.class, "frMotor"),
                hardwareMap.get(DcMotorEx.class, "blMotor"),
                hardwareMap.get(DcMotorEx.class, "brMotor")
        );

        CollectorMove = new CLift(
                hardwareMap.get(DcMotor.class, "cLift"),
                hardwareMap.get(RevTouchSensor.class, "TopLimit"),
                hardwareMap.get(RevTouchSensor.class, "BottomLimit")
        );
        //LAK modified 3_29_2021 DcMotor EX
        shooterCommands = new ShooterCommands(
                hardwareMap.get(DcMotorEx.class, "Shooter"),
                hardwareMap.get(Servo.class, "Platform"),
                hardwareMap.get(Servo.class, "lLift"),
                hardwareMap.get(Servo.class, "rLift"),
                hardwareMap.get(Servo.class, "Finger"),
                hardwareMap.get(Servo.class, "Finger2")
        );


        Shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Collector = hardwareMap.get(DcMotor.class, "Collector");
        FinalRoller = hardwareMap.get(Servo.class, "FinalRoller");
        Roller1 = hardwareMap.get(Servo.class, "Trigger");
        Roller = hardwareMap.get(Servo.class, "Roller");
        cLift = hardwareMap.get(DcMotor.class, "cLift");
        Platform = hardwareMap.get(Servo.class, "Platform");
        lLift = hardwareMap.get(Servo.class, "lLift");
        rLift = hardwareMap.get(Servo.class, "rLift");
        Trigger = hardwareMap.get(Servo.class, "Trigger");
        TopLimit = hardwareMap.get(RevTouchSensor.class, "TopLimit");
        BottomLimit = hardwareMap.get(RevTouchSensor.class, "BottomLimit");
        WobbleServo = hardwareMap.get(Servo.class, "WobbleServo");
        //   digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
        //   sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensorColorRange");
        double lServopos = .98;
        double rServopos = 0;
        lLift.setPosition(lServopos);
        rLift.setPosition(rServopos);
        double LastLoop;
        double ColorZero = 0;
        double ColorZero1 = 0;
        for (int i = 0; i < 1; i++){
            ColorZero += REVColor.blue() + REVColor.red() + REVColor.green();
            ColorZero1 += REVColor1.blue() + REVColor1.red() + REVColor1.green();

        }
        ColorZero /= 100;
        ColorZero1 /= 100;
        double Location;
        Location = 0;
        double FingerPos = 0.4;
        RFinger.setPosition(0.26);
        LFinger.setPosition(0.05);
        double EncoderTarget;



        telemetry.addLine("not on line");
        telemetry.addData("color add",REVColor1.blue() + REVColor1.red() + REVColor1.green() + REVColor.blue() + REVColor.red() + REVColor.green()- ColorZero - ColorZero1);
        telemetry.update();
        WobbleServo.setPosition(0);

        initVuforia();
        initTfod();
        while (true){
            telemetry.addLine("Collector Down");
            telemetry.update();
            if (CollectorMove.fullDown()){
                telemetry.addLine("break");
                telemetry.update();
                break;
            }
        }

        // WARNING:
        // In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
        // This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
        // CONSEQUENTLY do not put any driving commands in this loop.
        // To restore the normal opmode structure, just un-comment the following line:

        // Note: To use the remote camera preview:
        // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
        // Tap the preview window to receive a fresh image.

        try {
            telemetry.addData("Status", "Initialized");
            telemetry.addData(">", "Press Play to start");
            telemetry.update();
            waitForStart();
            telemetry.clear();
            telemetry.addData(">", "Started...Press 'A' to capture frame");
            telemetry.update();
            double ColorAverage;
            double ColorAverage1;

            //targetsUltimateGoal.activate();
            /*
             * Activate TensorFlow Object Detection before we wait for the start command.
             * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
             **/
            if (tfod != null) {
                tfod.activate();

                // The TensorFlow software will scale the input images from the camera to a lower resolution.
                // This can result in lower detection accuracy at longer distances (> 55cm or 22").
                // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
                // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
                // should be set to the value of the images used to create the TensorFlow Object Detection model
                // (typically 16/9).
                tfod.setZoom(2.5, 16.0/9.0);
            }

            while (opModeIsActive()) {
                telemetry.addLine("op mode active");
                telemetry.update();

                for (int j = 0; j < 4000; j++) {
                    if (tfod != null) {
                        // getUpdatedRecognitions() will return null if no new information is available since
                        // the last time that call was made.
                        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                        if (updatedRecognitions != null) {
                            telemetry.addData("# Object Detected", updatedRecognitions.size());
                            if (updatedRecognitions.size() == 0) {
                                // empty list.  no objects recognized.
                                telemetry.addData("TFOD", "No items detected.");
                                telemetry.addData("Target Zone", "A");
                                Location = 0;
                            } else {
                                // list is not empty.
                                // step through the list of recognitions and display boundary info.
                                int i = 0;
                                for (Recognition recognition : updatedRecognitions) {
                                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                            recognition.getLeft(), recognition.getTop());
                                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                            recognition.getRight(), recognition.getBottom());

                                    // check label to see which target zone to go after.
                                    if (recognition.getLabel().equals("Single")) {
                                        telemetry.addData("Target Zone", "B");
                                        Location = 1;
                                    } else if (recognition.getLabel().equals("Quad")) {
                                        telemetry.addData("Target Zone", "C");
                                        Location = 2;
                                    } else {
                                        telemetry.addData("Target Zone", "UNKNOWN");
                                    }
                                }
                            }
                            telemetry.addData("j", j);
                            telemetry.update();
                        }
                        sleep(1);
                    }
                    if (Location == 1) {
                        telemetry.addLine("Pos 2");
                        telemetry.update();
                        break;
                    }
                    if (Location == 2) {
                        telemetry.addLine("Pos 3");
                        telemetry.update();
                        break;
                    }
                }

                if (Location == 0) {
                    while (! CollectorMove.fullUp()) {}

                    Shooter.setPower(.95);
                    Collector.setPower(.95);
                    FinalRoller.setPosition(0);
                    Roller1.setPosition(1);
                    Roller.setPosition(1);
                    driveMotors.forward(41);
                    sleep(1500);
                    //LAK added telemetry
                    telemetry.setAutoClear(false);
                    telemetry.addLine();
                    telemetry.addData("sspd after drive", Shooter.getVelocity());
                    telemetry.update();

                    shooterCommands.pos4();
                    sleep(100);

                    telemetry.addLine();
                    telemetry.addData("sspd before shot 1", Shooter.getVelocity());
                    telemetry.update();

                    shooterCommands.shoot();
                    telemetry.addLine();
                    telemetry.addData("sspd after shot 1", Shooter.getVelocity());
                    telemetry.update();

                    sleep(100);

                    telemetry.addLine();
                    telemetry.addData("sspd before shot 2", Shooter.getVelocity());
                    telemetry.update();

                    shooterCommands.shoot();

                    telemetry.addLine();
                    telemetry.addData("sspd after shot 2", Shooter.getVelocity());
                    telemetry.update();

                    sleep(100);

                    telemetry.addLine();
                    telemetry.addData("sspd before shot 3", Shooter.getVelocity());
                    telemetry.update();

                    shooterCommands.shoot();

                    telemetry.addLine();
                    telemetry.addData("sspd after shot 3", Shooter.getVelocity());
                    telemetry.update();

                    sleep(2000);
                    driveMotors.forwardFAST(33);
                    driveMotors.leftFast(10);
                    WobbleServo.setPosition(1);

                    driveMotors.right(30);
                }

                if (Location == 1) {
                    while (! CollectorMove.fullUp()) {}

                    Shooter.setPower(.95);
                    Collector.setPower(.95);
                    FinalRoller.setPosition(0);
                    Roller1.setPosition(1);
                    Roller.setPosition(1);
                    driveMotors.forward(41);
                    sleep(1500);
                    shooterCommands.pos4();
                    sleep(100);
                    shooterCommands.shoot();
                    sleep(300);
                    shooterCommands.shoot();
                    sleep(300);
                    shooterCommands.shoot();

                    while (! CollectorMove.fullDown()) {}

                    while (! CollectorMove.fullUp()) {}

                    sleep(3000);
                    Shooter.setPower(.85);
                    sleep(300);
                    shooterCommands.shoot();

                    driveMotors.forwardFAST(60);
                    sleep(100);
                    driveMotors.rightFAST(15);
                    WobbleServo.setPosition(1);
                    driveMotors.rightFAST(20);
                    driveMotors.backFAST(24);
                }

                if (Location == 2) {
                    while (! CollectorMove.fullUp()) {}

                    Shooter.setPower(.95);
                    Collector.setPower(.95);
                    FinalRoller.setPosition(0);
                    Roller1.setPosition(1);
                    Roller.setPosition(1);
                    driveMotors.forward(41);
                    sleep(1500);
                    shooterCommands.pos4();
                    sleep(100);
                    shooterCommands.shoot();
                    sleep(100);
                    shooterCommands.shoot();
                    sleep(100);
                    shooterCommands.shoot();

                    CollectorMove.down(1100, 0.95);
                    sleep(500);
                    while (! CollectorMove.fullUp()) {}

                    sleep(3000);
                    Shooter.setPower(0);
                    sleep(50);
                    Shooter.setPower(.95);
                    sleep(50);
                    shooterCommands.shoot();

                    CollectorMove.down(1500, 0.95);
                    sleep(100);
                    while (! CollectorMove.fullUp()) {}

                    sleep(3000);
                    shooterCommands.shoot();
                    sleep(100);

                    driveMotors.forwardFAST(82);
                    driveMotors.leftFast(10);
                    WobbleServo.setPosition(1);
                    sleep(1000);
                    driveMotors.rightFAST(10);
                    driveMotors.backFAST(45);
                }
                sleep(10000);
                break;
            }
        } finally {
            // Disable Tracking when we are done;
            //targetsUltimateGoal.deactivate();
            if (tfod != null) {
                tfod.shutdown();
            }
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        webcamName = hardwareMap.get(WebcamName.class, "webcam");
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = webcamName;
        parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
