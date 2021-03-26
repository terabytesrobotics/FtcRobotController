package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


@SuppressWarnings("ALL")
@TeleOp(name="Terabytes Robotics Tele-Op", group ="Real")
public class Teleop extends LinearOpMode {

    private static final int secondsPermissionTimeout = Integer.MAX_VALUE;
    private static final String TAG = "Webcam Sample";

    private Click oneClick;
    private RevColorSensorV3 REVColor1;
    private RevColorSensorV3 REVColor;
    private DcMotor flDcMotor;
    private DcMotor frDcMotor;
    private DcMotor blDcMotor;
    private DcMotor brDcMotor;
    private DcMotor Shooter;
    private DcMotor Collector;
    private DcMotor cLift;
    private Servo Platform;
    private Servo lLift;
    private Servo rLift;
    private Servo Roller1;
    private Servo Roller;
    private RevTouchSensor TopLimit;
    private RevTouchSensor BottomLimit;
    private Servo LFinger;
    private Servo RFinger;
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

    // Class Members
    private VuforiaLocalizer vuforia = null;

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    WebcamName webcamName = null;

    /*
     * tfod is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static float Blue_Top_Goal_Lservo_Setpos = .855f;
    private static float Blue_Top_Goal_Rservo_Setpos = .124f;
    private static float Blue_Top_Goal_LRAngle_Setpos = .475f;

    @SuppressLint("DefaultLocale")
    @Override public void runOpMode() {
        REVColor1 = hardwareMap.get(RevColorSensorV3.class,"REVColor1");
        REVColor = hardwareMap.get(RevColorSensorV3.class, "REVColor");
        flDcMotor = hardwareMap.get(DcMotor.class, "flMotor");
        flDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frDcMotor = hardwareMap.get(DcMotor.class, "frMotor");
        frDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        blDcMotor = hardwareMap.get(DcMotor.class, "blMotor");
        blDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brDcMotor = hardwareMap.get(DcMotor.class, "brMotor");
        brDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Shooter = hardwareMap.get(DcMotor.class, "Shooter");
        Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Collector = hardwareMap.get(DcMotor.class, "Collector");
        cLift = hardwareMap.get(DcMotor.class, "cLift");
        Platform = hardwareMap.get(Servo.class, "Platform");
        lLift = hardwareMap.get(Servo.class, "lLift");
        rLift = hardwareMap.get(Servo.class, "rLift");
        FinalRoller = hardwareMap.get(Servo.class, "FinalRoller");
        Roller1 = hardwareMap.get(Servo.class, "Trigger");
        Roller = hardwareMap.get(Servo.class, "Roller");
        LFinger = hardwareMap.get(Servo.class, "Finger");
        RFinger = hardwareMap.get(Servo.class, "Finger2");
        TopLimit = hardwareMap.get(RevTouchSensor.class, "TopLimit");
        BottomLimit = hardwareMap.get(RevTouchSensor.class, "BottomLimit");

        double lServopos = .98;
        double rServopos = 0;
        lLift.setPosition(lServopos);
        rLift.setPosition(rServopos);

        initTfod();

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
            waitForStart();
            telemetry.clear();
            telemetry.addData(">", "Started...Press 'A' to capture frame");

            double PlatformPositionX = 0.5;
            boolean Up;
            double UpYes;
            boolean Down;
            boolean ToggleCollector;
            boolean CButtonlock = false;
            int rollerDirectionNonOverride = 0;
            boolean xPrev = false;
            boolean xToggled = false;
            boolean TButtonlock = false;
            boolean Left;
            boolean Right;
            double ShooterStatus = 0;
            boolean SButtonlock = false;
            boolean ToggleShooter;
            double PowershotSetpointStatus = 0;
            boolean TogglePowerSet;
            boolean SetpointButtonlock = false;
            boolean TriggersOn = false;
            double FingerPos = 0.4;

            // Vars for ArcadeMode
            double ColorAverage;
            double FL;
            double FR;
            double BL;
            double BR;

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

                //* taking the average of Red Green and BLue for both sensors
                ColorAverage = REVColor.red() + REVColor.green() + REVColor.blue();

                /**
                 OLD Holo drive code
                 */
                telemetry.addData("BottomLimit",BottomLimit.getValue());
                double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
                double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
                double rightX = gamepad1.right_stick_x * -1;
                final double v1 = r * Math.cos(robotAngle) + rightX;
                final double v2 = r * Math.sin(robotAngle) - rightX;
                final double v3 = r * Math.sin(robotAngle) + rightX;
                final double v4 = r * Math.cos(robotAngle) - rightX;

                FL = -v1;
                FR = -v2;
                BL = -v3;
                BR = -v4;

                if(gamepad1.left_bumper){
                    FL /= 2;
                    FR /= 2;
                    BL /= 2;
                    BR /= 2;
                }

                flDcMotor.setPower(FL);
                frDcMotor.setPower(FR);
                blDcMotor.setPower(BL);
                brDcMotor.setPower(BR);
                telemetry.addData("fl",cLift.getCurrentPosition());


                // TODO: Factor out telemetry
                telemetry.addData("flMotor Power", flDcMotor.getPower());
                telemetry.addData("frMotor Power", frDcMotor.getPower());
                telemetry.addData("blMotor Power", blDcMotor.getPower());
                telemetry.addData("brMotor Power", blDcMotor.getPower());
                telemetry.addData("PlatformPos", Platform.getPosition());
                telemetry.addData("PlatformPosLR", lServopos);
                telemetry.addData("CollectorPos", rollerDirectionNonOverride);
                telemetry.addData("LFinger", LFinger.getPosition());
                telemetry.addData("RFinger", RFinger.getPosition());
                // Set Points

                // Powershots
                TogglePowerSet = gamepad2.right_bumper;
                if(SetpointButtonlock && !TogglePowerSet){
                    SetpointButtonlock = false;
                }
                if(TogglePowerSet && !SetpointButtonlock){
                    PowershotSetpointStatus += 1;
                    SetpointButtonlock = true;
                }
                if(PowershotSetpointStatus>3){
                    PowershotSetpointStatus = 0;
                }
                if(PowershotSetpointStatus == 1){
                    PlatformPositionX = (.416);
                    lServopos = (0.877);
                    rServopos = (.98 - .877);
                }
                if(PowershotSetpointStatus == 2){
                    PlatformPositionX =(.372);
                    lServopos = (0.89);
                    rServopos = (.98 - .885);
                }
                if(PowershotSetpointStatus == 3){
                    PlatformPositionX = (.332);
                    lServopos = (0.888);
                    rServopos = (.98 - .888);
                }

                telemetry.addData("Platform Position", PowershotSetpointStatus);

                // Top Goal
                if(gamepad2.left_bumper) {
                    PlatformPositionX = Blue_Top_Goal_LRAngle_Setpos;
                    lServopos = Blue_Top_Goal_Lservo_Setpos;
                    rServopos = Blue_Top_Goal_Rservo_Setpos;
                    PowershotSetpointStatus = 0;
                }

                //shooter code
                ToggleShooter = this.gamepad2.b;
                if (SButtonlock && !ToggleShooter){
                    SButtonlock = false;
                }
                if (ToggleShooter && !SButtonlock) {
                    ShooterStatus += .8;
                    SButtonlock = true;
                }
                if (ShooterStatus > .8 && ! SButtonlock) {
                    ShooterStatus = 0;
                    SButtonlock = true;
                }
                Shooter.setPower(ShooterStatus);

                // Platform "Left Right" code

                Left = this.gamepad2.dpad_left;
                Right = this.gamepad2.dpad_right;
                if (Left) {
                    PlatformPositionX += .008;
                    PowershotSetpointStatus = 0;
                }
                if (Right) {
                    PlatformPositionX -= .008;
                    PowershotSetpointStatus = 0;
                }

                if (PlatformPositionX > 0.7) {
                    PlatformPositionX = 0.7;
                }
                if (PlatformPositionX < 0.3) {
                    PlatformPositionX = 0.3;
                }

                Platform.setPosition(PlatformPositionX);

                //Platform "Up Down" code
                UpYes = 0;
                Down = this.gamepad2.dpad_up;
                Up = this.gamepad2.dpad_down;
                if (Up) {
                    UpYes = .004;
                    PowershotSetpointStatus = 0;
                }
                if (Down) {
                    UpYes = -.004;
                    PowershotSetpointStatus = 0;
                }

                lServopos += UpYes;
                rServopos -= UpYes;
                if (lServopos > 0.98) {
                    lServopos = 0.98;
                }
                if (lServopos < 0) {
                    lServopos = 0;
                }
                if (rServopos > .98) {
                    rServopos = .98;
                }
                if (rServopos < 0) {
                    rServopos = 0;
                }
                lLift.setPosition(lServopos);
                rLift.setPosition(rServopos);

                //Collector code

                int rollerDirection;
                boolean bReverseOverride = gamepad1.b;
                boolean xPressed = gamepad1.x;
                boolean xChanged = xPressed != xPrev;
                boolean xNewlyPressed = xPressed && xChanged;

                if (xNewlyPressed && !xToggled) {
                    xToggled = true;
                    rollerDirectionNonOverride = 1;
                } else if (xNewlyPressed && xToggled) {
                    xToggled = false;
                    rollerDirectionNonOverride = 0;
                }

                xPrev = xPressed;

                if (bReverseOverride) {
                    rollerDirection = -1;
                } else {
                    rollerDirection = rollerDirectionNonOverride;
                }

                Collector.setPower(rollerDirection);
                FinalRoller.setPosition((-rollerDirection + 0.5)/2);
                Roller1.setPosition(rollerDirection + 0.5);
                Roller.setPosition(rollerDirection + 0.5);

//                if (gamepad1.b) {
//                    Collector.setPower(1);
//                    FinalRoller.setPosition(0);
//                    Roller1.setPosition(1);
//                    Roller.setPosition(1);
//                }
//                else {
//                    Collector.setPower(-CollectorStatus);
//                    FinalRoller.setPosition(-CollectorStatus);
//                    Roller1.setPosition(-CollectorStatus);
//                    Roller.setPosition(-CollectorStatus);
//                }
//                ToggleCollector = this.gamepad1.x;
//                if (CButtonlock && !ToggleCollector){
//                    CButtonlock = false;
//                }
//                if (ToggleCollector && !CButtonlock) {
//                    CollectorStatus += 1;
//                    FinalRoller.setPosition(1);
//                    Roller1.setPosition(1);
//                    Roller.setPosition(1);
//                    CButtonlock = true;
//                }
//                if (CollectorStatus > 1 && !CButtonlock) {
//                    Roller1.setPosition(.5);
//                    FinalRoller.setPosition(.5);
//                    Roller.setPosition(.5);
//                    CollectorStatus = 0;
//                    CButtonlock = true;
//                }

                RFinger.setPosition(FingerPos);
                LFinger.setPosition(.31 - FingerPos);
                TriggersOn = this.gamepad2.y;
                if (TriggersOn){
                    FingerPos = 0;
                }else{
                    FingerPos = 0.26;
                }

                //* Collector Up and Down Code
                if (gamepad1.right_bumper && TopLimit.getValue() ==0){
                    cLift.setPower(-1);
                    telemetry.addLine("TopLimit.getValue() ==0");
                }
                else if (gamepad1.right_bumper && TopLimit.getValue() ==1){
                    cLift.setPower(0);
                    telemetry.addLine("TopLimit.getValue() ==1");
                }
                else if (BottomLimit.getValue() ==0) {
                    cLift.setPower(1);
                    telemetry.addLine("BottomLimit.getValue() ==0");
                }
                else{
                    cLift.setPower(0);
                    telemetry.addLine("none");
                }

                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                        }
                    }
                }

                telemetry.update();
            }
        } finally {

            if (tfod != null) {
                tfod.shutdown();
            }
        }
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
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
        parameters.useExtendedTracking = true;

        // Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
