package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


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
    public static double angleoffset = -28;

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

    private boolean aWasPressed = false;
    private boolean bPressed = false;
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

        greenGrabber.setPosition(.4);
        blueGrabber.setPosition(.8);
//arm and ex
        armcontrol = new PIDController(p, i, d);
        arm_motor0 = hardwareMap.get(DcMotorEx.class, "arm_motorE0");
        extender = hardwareMap.get(DcMotorEx.class, "extenderE1");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm_motor0.setDirection(DcMotorEx.Direction.FORWARD);
        extender.setDirection(DcMotorSimple.Direction.REVERSE);


        extender.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        extender.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        extender.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        //extender.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        extender.setTargetPosition(0);






        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        GrabberState blueGrabberState = GrabberState.NOT_GRABBED;
        GrabberState greenGrabberState = GrabberState.NOT_GRABBED;
        // run until the end of the match (driver presses STOP)
        if (opModeIsActive()) {

            while (opModeIsActive()) {

                if (gamepad1.a) {
                    if (aWasPressed) {
                        blueGrabberState = blueGrabberState.toggle();
                        greenGrabberState = greenGrabberState.toggle();
                    }
                    aWasPressed = true;
                } else {
                    aWasPressed = false;
                }

                /*if (gamepad1.a && !aPressed) {
                    aPressed = true;

                    lastAPressTime = System.currentTimeMillis();
                    redGrabberState = GrabberState.GRABBED;
                */
                    /*
                    if ((int)greenGrabber.getPosition()*100 == 60) {
                        greenGrabber.setPosition(1);
                    }
                    else if (greenGrabber.getPosition() == 1) {
                        greenGrabber.setPosition(.6);*/
                /*f (!gamepad1.a) {
                    aPressed = false;
                }*/

                // Debounce for gamepad1.b
/*                if (gamepad1.b && !bPressed && (System.currentTimeMillis() - lastBPressTime) > DEBOUNCE_TIME) {
                    bPressed = true;
                    lastBPressTime = System.currentTimeMillis();

                    if (blueGrabber.getPosition() == 0.575) {
                        blueGrabber.setPosition(1);
                    }
                } else if (!gamepad1.b) {
                    bPressed = false;
                }*/

                switch (greenGrabberState) {
                    case GRABBED:
                        greenGrabber.setPosition(.4);
                        break;
                    case NOT_GRABBED:
                        greenGrabber.setPosition(.1);
                        break;
                }

                switch (blueGrabberState) {
                    case GRABBED:
                        blueGrabber.setPosition(.8);
                        break;
                    case NOT_GRABBED:
                        blueGrabber.setPosition(.5);
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
                //arm_motor0.setPower(armpower);
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
}


