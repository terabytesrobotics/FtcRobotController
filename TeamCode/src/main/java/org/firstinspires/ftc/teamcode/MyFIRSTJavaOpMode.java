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
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

//*i made a change
@TeleOp(name="Terabytes Robotics Tele-Op", group ="Real")

public class MyFIRSTJavaOpMode extends LinearOpMode {
    private static final int secondsPermissionTimeout = Integer.MAX_VALUE;
    private static final String TAG = "Webcam Sample";
  //  private Gyroscope imu;
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
    private Servo Trigger;
    private RevTouchSensor TopLimit;
    private RevTouchSensor BottomLimit;
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
    private static final float halfField = 71 * mmPerInch;
    private static final float quadField  = 35.5f * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private VuforiaTrackables targetsUltimateGoal = null;
    private List<VuforiaTrackable> allTrackables = null;

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    WebcamName webcamName = null;

    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;
    private float RX;
    private float RY;
    private float RZ;
    /*
     * tfod is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static float Blue_Top_Goal_Lservo_Setpos = .864f;
    private static float Blue_Top_Goal_Rservo_Setpos = .136f;
    private static float Blue_Top_Goal_LRAngle_Setpos = .5f;




    @SuppressLint("DefaultLocale")
    @Override public void runOpMode() {
    //    imu = hardwareMap.get(Gyroscope.class, "IMU");
        //*REVColor1 =
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
        Trigger = hardwareMap.get(Servo.class, "Trigger");
        TopLimit = hardwareMap.get(RevTouchSensor.class, "TopLimit");
        BottomLimit = hardwareMap.get(RevTouchSensor.class, "BottomLimit");
        //   digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
        //   sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensorColorRange");
        double lServopos = .98;
        double rServopos = 0;
        lLift.setPosition(lServopos);
        rLift.setPosition(rServopos);



        initVuforia();
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
            telemetry.update();
            waitForStart();
            telemetry.clear();
            telemetry.addData(">", "Started...Press 'A' to capture frame");
            telemetry.update();


            double XtgtPower;
            double LYtgtPower;
            double RYtgtPower;
            double PlatformPositionX;
            double PlatformMap;
            boolean Shoot;
            double ShootYes;
            boolean Up;
            double UpYes;
            boolean Down;
            boolean ToggleCollector;
            boolean CButtonlock;
                CButtonlock = false;
            double CollectorStatus;
            CollectorStatus = 0;
            boolean ToggleTrigger;
            double TriggerStatus;
            TriggerStatus = .5;
            boolean TButtonlock;
                TButtonlock = false;
            boolean Trigger_yes;
            boolean Collector_Up;

            boolean Left;
            boolean Right;
            double LRYes;
            double ShooterStatus;
            boolean SButtonlock;
            SButtonlock = false;
            boolean ToggleShooter;
            ShooterStatus = 0;

            float AngleX;
            float AngleY;
            double TArad;
            double ServoControlAngle;
            ServoControlAngle = 4.01426;
            double ServoCenter;
            ServoCenter = 0.5;
            double TargetAngleSERVO;
                TargetAngleSERVO = 0.5;

            // Vars for ArcadeMode
            double ColorAverage = 0;
            double ColorAverage1 = 0;
            double ColorOffset = 40;
            double MattColor = 100;
            double ColorOffsetSlow = 50;
            double ColorOffsetStop = 100;
            double fakeR = 0;
            double fakeY;
                fakeY = 0;
            boolean ArcadeMode;
                ArcadeMode = false;
            boolean color0;
                color0 = false;
            boolean color1;
                color1 = false;
            double fakespeed;
                fakespeed = 0;
            double ColorDifference = 0;
            double ColorRotateTrim =60;
            double ColorRotate = 0;

            double FL = 0;
            double FR = 0;
            double BL = 0;
            double BR = 0;
            targetsUltimateGoal.activate();
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
            PlatformPositionX = 0.5;
            while (opModeIsActive()) {

                //* taking the average of Red Green and BLue for both sensors
                ColorAverage = REVColor.red() + REVColor.green() + REVColor.blue();
                //*ColorAverage1 = REVColor1.red() + REVColor1.green() + REVColor1.blue();

                /*
                 OLD Holo drive code
                 */
/*
                double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
                double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
                double rightX = gamepad1.right_stick_x * -1;
                final double v1 = r * Math.cos(robotAngle) + rightX;
                final double v2 = r * Math.sin(robotAngle) - rightX;
                final double v3 = r * Math.sin(robotAngle) + rightX;
                final double v4 = r * Math.cos(robotAngle) - rightX;
   */

                //* New Drive Code



                //* deciding if Arcade mode is active and setting "ArcadeMode"
                ArcadeMode = gamepad1.right_bumper;

                //* Choosing the speed that the robot should be going based on the color sensor values setting that to fakespeed
                if(ColorAverage >= ColorOffsetStop + MattColor || ColorAverage1  >= ColorOffsetStop + MattColor ){
                    fakespeed = 0;
                }
                else if(ColorAverage >= ColorOffsetSlow + MattColor|| ColorAverage1  >= ColorOffsetSlow + MattColor ){
                    fakespeed = 0.2;
                }
                else{
                    fakespeed = 1;
                }
                ColorDifference = ColorAverage -ColorAverage1;
                //*deciding if the Robot should rotate
                if (ColorDifference >= ColorRotateTrim){
                    ColorRotate = 1;
                }
                else{
                    ColorRotate = 0;
                }




                //Locking the driver out and giving control to fakespeed and fakeRotate
                if (!ArcadeMode){
                    fakeY = gamepad1.left_stick_y;
                    fakeR = gamepad1.right_stick_x;
                }
                else {
                        fakeY = fakespeed;
                        fakeR = ColorRotate;
                    }


                double r = Math.hypot(-gamepad1.left_stick_x, fakeY);
                double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
                double rightX = fakeR * -1;
                final double v1 = r * Math.cos(robotAngle) + rightX;
                final double v2 = r * Math.sin(robotAngle) - rightX;
                final double v3 = r * Math.sin(robotAngle) + rightX;
                final double v4 = r * Math.cos(robotAngle) - rightX;

                FL = -v1;
                FR = -v2;
                BL = v3;
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


             //   telemetry.addData("Switchtest", BottomLimit.getValue());
             //   telemetry.addData("Switchtest2", TopLimit.getValue());

             /*   XtgtPower = -this.gamepad1.left_stick_x;
                LYtgtPower = -this.gamepad1.left_stick_y;
                RYtgtPower = this.gamepad1.left_stick_y;
                PlatformPositionX = this.gamepad2.right_stick_x;
                if (PlatformPositionX < -0.3) {
                    PlatformPositionX = -0.3;
                }
                if (PlatformPositionX > 0.3) {
                    PlatformPositionX = 0.3;
                }
                PlatformMap = (-PlatformPositionX + 0.5); */
             /*   flDcMotor.setPower(-(XtgtPower - LYtgtPower));
                blDcMotor.setPower(XtgtPower - LYtgtPower);
                frDcMotor.setPower(XtgtPower - RYtgtPower);
                brDcMotor.setPower(XtgtPower - RYtgtPower);
                telemetry.addData("X Target Power", XtgtPower);
                telemetry.addData("flMotor Power", flDcMotor.getPower());
                telemetry.addData("frMotor Power", frDcMotor.getPower());
                telemetry.addData("blMotor Power", blDcMotor.getPower());
                telemetry.addData("brMotor Power", blDcMotor.getPower());
             */   //while(ServoMap<1) {
                 //   Platform.setPosition(PlatformMap);
                //}
                telemetry.addData("PlatformPos", Platform.getPosition());
                telemetry.addData("PlatformPosLR", lServopos);
               // telemetry.addData("PlatformMap", PlatformMap);
                telemetry.addData("CollectorPos", CollectorStatus);
/*

*/



                if(gamepad2.left_bumper){
                    PlatformPositionX = Blue_Top_Goal_LRAngle_Setpos;
                    lServopos = Blue_Top_Goal_Lservo_Setpos;
                    rServopos = Blue_Top_Goal_Rservo_Setpos;
                }

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
                //Platform "Left Right" code

                Left = this.gamepad2.dpad_left;
                Right = this.gamepad2.dpad_right;
                if (Left) {
                    PlatformPositionX += .008;
                }
                if (Right) {
                    PlatformPositionX -= .008;
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
                }
                if (Down) {
                    UpYes = -.004;
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


                Collector.setPower(CollectorStatus);
                ToggleCollector = this.gamepad1.x;
                if (CButtonlock && !ToggleCollector){
                    CButtonlock = false;
                }
                if (ToggleCollector && !CButtonlock) {
                    CollectorStatus += 1;
                    CButtonlock = true;
                }
                if (CollectorStatus > 1 && ! CButtonlock) {
                    CollectorStatus = 0;
                    CButtonlock = true;
                }

                //Trigger code
                Trigger.setPosition(TriggerStatus);
                ToggleTrigger = this.gamepad2.y;
                if (TButtonlock && !ToggleTrigger){
                    TButtonlock = false;
                }
                if (TriggerStatus == 0.5 && ToggleTrigger && !TButtonlock) {
                    TriggerStatus = 0;
                    TButtonlock = true;
                }
                if (TriggerStatus == 0 && ! TButtonlock && ToggleTrigger) {
                    TriggerStatus = 0.5;
                    TButtonlock = true;
                }

                //* Collector Up and Down Code


                if (gamepad1.right_bumper && TopLimit.getValue() ==0){
                    cLift.setPower(-1);
                }
                else if (gamepad1.right_bumper && TopLimit.getValue() ==1){
                    cLift.setPower(0);
                }
                else if (BottomLimit.getValue() ==0) {
                    cLift.setPower(1);
                }
                else{
                    cLift.setPower(0);
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
                            //*telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            //*telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            //*        recognition.getLeft(), recognition.getTop());
                            //*telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            //*        recognition.getRight(), recognition.getBottom());
                        }
                        telemetry.update();
                    }
                }

                // check all the trackable targets to see which one (if any) is visible.
                targetVisible = false;
                for (VuforiaTrackable trackable : allTrackables) {
                    if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                        telemetry.addData("Visible Target", trackable.getName());
                        targetVisible = true;

                        // getUpdatedRobotLocation() will return null if no new information is available since
                        // the last time that call was made, or if the trackable is not currently visible.
                        OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                        if (robotLocationTransform != null) {
                            lastLocation = robotLocationTransform;
                        }
                        break;
                    }
                }

                // Provide feedback as to where the robot is located (if we know).
                if (targetVisible) {
                    // express position (translation) of robot in inches.
                    VectorF translation = lastLocation.getTranslation();
                   //* telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                   //*         translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);


                            // express the rotation of the robot in degrees.
                    Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                   //* telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                    RX = translation.get(0) /mmPerInch;
                    RY = translation.get(1) /mmPerInch;
                    RZ = translation.get(2) /mmPerInch;




                } else {
                    telemetry.addData("Visible Target", "none");
                }
                telemetry.addData("Status", "Running");
                telemetry.update();
                telemetry.addData("RX =",RX);
                telemetry.addData("RY =",RY);
                telemetry.addData("RZ =",RZ);





                //* Auto aim Code
             /*   if (gamepad2.x){

                    AngleX = 72 - RX;
                    AngleY = 36 - RY;

                    TArad = (atan(AngleY/AngleX));
                    TargetAngleSERVO = TArad / ServoControlAngle + ServoCenter;
                    Platform.setPosition(TargetAngleSERVO);


                }
                telemetry.addData("shooter angle",TargetAngleSERVO);
*/



            }
        } finally {
            // Disable Tracking when we are done;
            targetsUltimateGoal.deactivate();

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
        parameters.useExtendedTracking = true;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");
        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal Target");
        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");
        VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
        redAllianceTarget.setName("Red Alliance Target");
        VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
        blueAllianceTarget.setName("Blue Alliance Target");
        VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
        frontWallTarget.setName("Front Wall Target");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables = new ArrayList<>();
        allTrackables.addAll(targetsUltimateGoal);

        /*
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        //Set the position of the perimeter targets with relation to origin (center of field)
        redAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        blueAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        frontWallTarget.setLocation(OpenGLMatrix
                .translation(-halfField, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
        blueTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
        redTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }
        //phoneYRotate = 0;

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90;
        }
        //phoneXRotate = 0;

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT = 8.5f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 6.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 0.0f;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /*  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }
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
