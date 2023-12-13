package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.OnActivatedEvaluator;


@Config
@TeleOp

public class TeleOpNibus2000 extends LinearOpMode {
    private DcMotor leftFrontDrive = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive = null;  //  Used to control the right back drive wheel
    private ElapsedTime runtime = new ElapsedTime();

    private TouchSensor armMin;
    private TouchSensor extenderMin;
    private PIDController armcontrol;

    public static double p = 0.005, i = 0.005, d = 0.0002;
    public static double f = 0.0;

    // Angle below horiontal at start in degrees.  horizontal is 0.
    public static double angleoffset = -37;

    //Max angle limit
    public static double maxangle = 180;

    //Max arm extension in cm
    public static double maxextend = 19;
    public static int extendTolerance = 4;

    public static double extendPower = 0.8;

    //Length to extend in cm
    public static double extendLength = 0;

    public static int extendTicTarget = 0;

    public double fmax = 0;

    public static double extendergearratio = (5.2);


    static double extender_tics_per_cm = extendergearratio * 28 / 0.8;

    public static double degtarget = 0;
    public static double target = 0.0;

    public static double armTolerance = 10;

    private final double gear_ratio = 13.7;

    private static double worm_ratio = 28;

    private final double arm_ticks_per_degree = worm_ratio * 28 * gear_ratio / 360;

    private DcMotorEx arm_motor0;


    private DcMotorEx extender;

    //collection servos
    Servo greenGrabber;

    Servo blueGrabber;

    Servo wrist;

    private CollectorState collectorState = CollectorState.CLOSE_COLLECTION;

    private double collectorHeight = 0;

    private static final long DEBOUNCE_TIME = 500; // Debounce time in milliseconds

    private enum BlueGrabberState {
        GRABBED(0.65f),
        NOT_GRABBED(0.9f);

        public float ServoPosition;

        private BlueGrabberState(float servoPosition) {
            ServoPosition = servoPosition;
        }

        public BlueGrabberState toggle() {
            switch (this) {
                case NOT_GRABBED:
                    return GRABBED;
                case GRABBED:
                    return NOT_GRABBED;
                default:
                    return GRABBED;
            }
        }
    }

    private enum GreenGrabberState {
        GRABBED(0.3f),
        NOT_GRABBED(0.02f);

        public float ServoPosition;

        private GreenGrabberState(float servoPosition) {
            ServoPosition = servoPosition;
        }

        public GreenGrabberState toggle() {
            switch (this) {
                case NOT_GRABBED:
                    return GRABBED;
                case GRABBED:
                    return NOT_GRABBED;
                default:
                    return GRABBED;
            }
        }
    }

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
    public void runOpMode() {
        //Wheel Drive Motors
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
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        greenGrabber.setPosition(0);
        blueGrabber.setPosition(.8);
//arm and ex
        armMin = hardwareMap.get(TouchSensor.class, "armMin1");
        extenderMin = hardwareMap.get(TouchSensor.class, "extenderMin3");
        armcontrol = new PIDController(p, i, d);
        arm_motor0 = hardwareMap.get(DcMotorEx.class, "arm_motorE0");
        arm_motor0.setDirection(DcMotorEx.Direction.FORWARD);
        wrist.setPosition(.8);

        OnActivatedEvaluator a1PressedEvaluator = new OnActivatedEvaluator(() -> gamepad1.a);
        OnActivatedEvaluator lb1PressedEvaluator = new OnActivatedEvaluator(() -> gamepad1.left_bumper);
        OnActivatedEvaluator rb1PressedEvaluator = new OnActivatedEvaluator(() -> gamepad1.right_bumper);
        OnActivatedEvaluator a2PressedEvaluator = new OnActivatedEvaluator(() -> gamepad2.a);
        OnActivatedEvaluator b2PressedEvaluator = new OnActivatedEvaluator(() -> gamepad2.b);
        OnActivatedEvaluator x2PressedEvaluator = new OnActivatedEvaluator(() -> gamepad2.x);
        OnActivatedEvaluator y2PressedEvaluator = new OnActivatedEvaluator(() -> gamepad2.y);
        OnActivatedEvaluator lb2PressedEvaluator = new OnActivatedEvaluator(() -> gamepad2.left_bumper);
        OnActivatedEvaluator rb2PressedEvaluator = new OnActivatedEvaluator(() -> gamepad2.right_bumper);



        extender = hardwareMap.get(DcMotorEx.class, "extenderE1");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        extender.setDirection(DcMotorSimple.Direction.FORWARD);
        extender.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        extender.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        extender.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        //extender.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);


        autoHomeCollectorLoop();
        arm_motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        degtarget = 0;
        extender.setTargetPosition(0);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        BlueGrabberState blueGrabberState = BlueGrabberState.NOT_GRABBED;
        GreenGrabberState greenGrabberState = GreenGrabberState.NOT_GRABBED;

        //Test for set points collecting
        //GrabberState closeSetPoint = GrabberState.NOT_GRABBED;
        //end of test

        // run until the end of the match (driver presses STOP)
        if (opModeIsActive()) {

            while (opModeIsActive()) {
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

                //Test for set points collecting
                if (a2PressedEvaluator.evaluate()) {
                    collectorState = CollectorState.CLOSE_COLLECTION;
                } else if (b2PressedEvaluator.evaluate()) {
                    collectorState = CollectorState.FAR_COLLECTION;
                } else if (x2PressedEvaluator.evaluate()) {
                    collectorState = CollectorState.DRIVING_SAFE;
                } else if (y2PressedEvaluator.evaluate()) {
                    collectorState = CollectorState.LOW_SCORING;
                } else if (rb2PressedEvaluator.evaluate()) {
                    collectorState = CollectorState.FAR_COLLECTION;
                } else if (lb2PressedEvaluator.evaluate()) {
                    collectorState = CollectorState.SAFE_POSITION;
                }
                telemetry.addData("Collector state", collectorState);

                degtarget = collectorState.ArmPosition;
                extendLength = collectorState.ExtenderPosition;
                wrist.setPosition(collectorState.WristPosition);
                collectorHeight = 0;
                telemetry.addData("Wrist pos:",wrist.getPosition());
                greenGrabber.setPosition(greenGrabberState.ServoPosition);
                blueGrabber.setPosition(blueGrabberState.ServoPosition);

                target = ((Math.min(degtarget, maxangle)) - angleoffset) * arm_ticks_per_degree;

                armcontrol.setPID(p, i, d);

                armcontrol.setTolerance(armTolerance);


                int armPos = arm_motor0.getCurrentPosition();
                int extendPos = extender.getCurrentPosition();

                double pid = armcontrol.calculate(armPos, target);

                double ff = ((f) + (fmax * extendPos / (maxextend * extender_tics_per_cm))) * Math.cos(Math.toRadians(degtarget));


                double armpower = pid + ff;
                //Set extension
                extendTicTarget = (int) ((Math.min(extendLength, maxextend)) * extender_tics_per_cm);
                extender.setTargetPosition(extendTicTarget);
                extender.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);


                // RIGHT HERE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                if(armpower < 0 && armMin.isPressed()) armpower = 0;
                arm_motor0.setPower(armpower);
               // if(extender.get < 0 && extenderMin.isPressed()){
                  //  armpower = 0;
                  //  extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                   // extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //}
                extender.setPower(extendPower);

                telemetry.addData("f", f);
                telemetry.addData("ff", ff);
                telemetry.addData("Armpower", armpower);
                telemetry.addData("Position", armPos);
                telemetry.addData("target", target);
                telemetry.addData("AnglePos", (armPos / arm_ticks_per_degree) + angleoffset);
                telemetry.addData("angle target", degtarget);
                telemetry.addData("Extender Tic Target", extendTicTarget);
                telemetry.addData("Extender Current Tics", extendPos);
                telemetry.addData("Extender Target Length cm", extendLength);
                telemetry.addData("Extend Current Length", extendPos / extender_tics_per_cm);
                telemetry.addData("extender power", extender.getPower());
                telemetry.addData("green",greenGrabber.getPosition());
                telemetry.addData("blue",blueGrabber.getPosition());
                telemetry.update();
            }
        }
    }

    private double[] triangleCalculator(double sideA, double sideB, double angleC) {
        double[] output = new double[3];
        //output[0] = sideC. c^2 = a^2 + b^2 - 2abcos(C)
        output[0] = Math.sqrt(Math.pow(sideA, 2) + Math.pow(sideB, 2) - 2 * sideA * sideB * Math.cos(Math.toRadians(angleC)));
        //output[1] = angleA. sin(A)/a = sin(B)/b
        telemetry.addData("sideA", sideA);
        output[1] = 200 - Math.toDegrees(Math.asin((sideB * Math.sin(Math.toRadians(angleC))) / output[0]));
        //output[2] = angleB. 180 - angleC - angleA
        telemetry.addData("sideB", sideB);
        output[2] = Math.toDegrees(Math.asin((sideA * Math.sin(Math.toRadians(angleC))) / output[0]));
        telemetry.addData("angleC", angleC);
        return output;
    }
    private void autoHomeCollectorLoop() {
        //home extender
        extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (!extenderMin.isPressed()){
            extender.setPower(-0.4);
        }
        while (extenderMin.isPressed()){
            extender.setPower(0.4);
        }
        sleep(500);

        while (!extenderMin.isPressed()){
            extender.setPower(-0.1);
        }
        extender.setPower(0);
        extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //home arm
        while (!armMin.isPressed()){
            arm_motor0.setPower(-0.3);
        }
        while (armMin.isPressed()){
            arm_motor0.setPower(0.4);
        }
        sleep(500);

        while (!armMin.isPressed()){
            arm_motor0.setPower(-0.05);
        }
        arm_motor0.setPower(0);
        arm_motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}


