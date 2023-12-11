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
import com.qualcomm.robotcore.util.ElapsedTime;


@Config
@TeleOp

public class TeleOpNibus2000 extends LinearOpMode {
    private DcMotor leftFrontDrive = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive = null;  //  Used to control the right back drive wheel
    private ElapsedTime runtime = new ElapsedTime();

    private PIDController armcontrol;

    public static double p = 0.005, i = 0.005, d = 0.0002;
    public static double f = 0.0;

    // Angle below horiontal at start in degrees.  horizontal is 0.
    public static double angleoffset = 0;

    //Max angle limit
    public static double maxangle = 135;

    //Max arm extension in cm
    public static double maxextend = 20;
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

    private boolean aWasPressed = false;


    private boolean lBWasPressed = false;
    private boolean rBWasPressed = false;
    private boolean bPressed = false;
    private int collectorState = 0;
    private boolean twoAWasPressed = false;
    private boolean twoBWasPressed = false;
    private boolean twoXWasPressed = false;
    private boolean twoYWasPressed = false;

    private double collectorHeight = 0;

    private long lastAPressTime = 0;
    private long lastBPressTime = 0;
    private static final long DEBOUNCE_TIME = 500; // Debounce time in milliseconds

    private enum GrabberState {
        GRABBED,
        NOT_GRABBED;

        public GrabberState toggle() {
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

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        GrabberState blueGrabberState = GrabberState.NOT_GRABBED;
        GrabberState greenGrabberState = GrabberState.NOT_GRABBED;

        //Test for set points collecting
        GrabberState closeSetPoint = GrabberState.NOT_GRABBED;
        //end of test

        // run until the end of the match (driver presses STOP)
        if (opModeIsActive()) {

            while (opModeIsActive()) {


                //end of test

                if (gamepad1.a) {
                    if (!aWasPressed) {
                        blueGrabberState = blueGrabberState.toggle();
                        greenGrabberState = greenGrabberState.toggle();
                    }
                    aWasPressed = true;
                } else {
                    aWasPressed = false;
                }

                if (gamepad1.left_bumper) {
                    if (!lBWasPressed) {

                        greenGrabberState = greenGrabberState.toggle();
                    }
                    lBWasPressed = true;
                } else {
                    lBWasPressed = false;
                }

                if (gamepad1.right_bumper) {
                    if (!rBWasPressed) {

                        blueGrabberState = blueGrabberState.toggle();
                    }
                    rBWasPressed = true;
                } else {
                    rBWasPressed = false;
                }

                //Test for set points collecting
                if(gamepad2.a) collectorState = 1;
                if(gamepad2.b) collectorState = 2;
                if(gamepad2.x) collectorState = 3;
                if(gamepad2.y) collectorState = 4;
                if(gamepad2.right_bumper) collectorState = 5;
                if(gamepad2.left_bumper) collectorState = 6;
                telemetry.addData("Collector state",collectorState);
                switch (collectorState) {
                    case 1:
                        degtarget = -19;
                        extendLength = 0;
                        wrist.setPosition(0);
                        collectorHeight = 0;
                        break;
                    case 2:
                        degtarget = -10;
                        extendLength = 18;
                        wrist.setPosition(0);
                        collectorHeight = 0;
                        break;
                    case 3:
                        degtarget = 0;
                        extendLength = 0;
                        wrist.setPosition(0);
                        collectorHeight = 0;
                        break;
                    case 4:
                        degtarget = 170;
                        extendLength =0 ;
                        wrist.setPosition(.3);
                        collectorHeight = 0;
                        break;
                    case 5:
                        degtarget = 110;
                        extendLength = 18;
                        wrist.setPosition(.3);
                        collectorHeight =0;
                        break;
                    case 6:
                        collectorHeight = collectorHeight + gamepad2.right_stick_y;
                        telemetry.addData("Collector Height", collectorHeight);
                        degtarget = triangleCalculator(collectorHeight, 10, 100)[1];
                        telemetry.addData("Angle", degtarget);
                        extendLength = triangleCalculator(collectorHeight, 10, 90)[0];
                        telemetry.addData("Extend Length", extendLength);
                        wrist.setPosition(0.5);
                        break;
                }


                switch (greenGrabberState) {
                    case GRABBED:
                        greenGrabber.setPosition(.3);
                        break;
                    case NOT_GRABBED:
                        greenGrabber.setPosition(0.02);
                        break;
                }

                switch (blueGrabberState) {
                    case GRABBED:
                        blueGrabber.setPosition(.65);
                        break;
                    case NOT_GRABBED:
                        blueGrabber.setPosition(.9);
                        break;
                }


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
                arm_motor0.setPower(armpower);
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
                telemetry.addData("a",aWasPressed);
                telemetry.addData("b",bPressed);
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
        output[1] = Math.toDegrees(Math.asin((sideB * Math.sin(Math.toRadians(angleC))) / output[0]));
        //output[2] = angleB. 180 - angleC - angleA
        output[2] = Math.toDegrees(Math.asin((sideA * Math.sin(Math.toRadians(angleC))) / output[0]));
        return output;
    }
}


