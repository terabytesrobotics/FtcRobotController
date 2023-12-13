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

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.BlueGrabberState;
import org.firstinspires.ftc.teamcode.util.GreenGrabberState;
import org.firstinspires.ftc.teamcode.util.OnActivatedEvaluator;

@Config
@TeleOp
public class TeleOpNibus2000 extends LinearOpMode {

    private TouchSensor extenderMin;
    public static final double ARM_TOLERANCE = 10d;

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

    private SampleMecanumDrive drive;
    private TouchSensor armMin;
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

    private CollectorState collectorState = CollectorState.CLOSE_COLLECTION;

    private OnActivatedEvaluator a1PressedEvaluator;
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
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        a1PressedEvaluator = new OnActivatedEvaluator(() -> gamepad1.a);
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

        greenGrabber = hardwareMap.get(Servo.class, "greenE0");
        blueGrabber = hardwareMap.get(Servo.class, "blueE1");
        wrist = hardwareMap.get(Servo.class, "redE3");

        armMin = hardwareMap.get(TouchSensor.class, "armMin1");
        extenderMin = hardwareMap.get(TouchSensor.class, "extenderMin3");
        armcontrol = new PIDController(p, i, d);
        arm_motor0 = hardwareMap.get(DcMotorEx.class, "arm_motorE0");
        arm_motor0.setDirection(DcMotorEx.Direction.FORWARD);
        wrist.setPosition(.8);

        while (!armMin.isPressed()){
            arm_motor0.setPower(-0.3);
        }
        while (armMin.isPressed()){
            arm_motor0.setPower(0.4);
        }
        sleep(500);

        while (!armMin.isPressed()){
            arm_motor0.setPower(-0.1);
        }

        arm_motor0.setPower(0);
        arm_motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armTargetDegrees = 0;

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
        armTargetDegrees = 0;
        extender.setTargetPosition(0);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (!isStopRequested()) {
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
            extender.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            if(armpower < 0 && armMin.isPressed()) armpower = 0;
            arm_motor0.setPower(armpower);
            extender.setPower(EXTENDER_POWER);
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
            //home arm
            while (!armMin.isPressed()) {
                arm_motor0.setPower(-0.3);
            }
            while (armMin.isPressed()) {
                arm_motor0.setPower(0.4);
            }
            sleep(500);

            while (!armMin.isPressed()) {
                arm_motor0.setPower(-0.05);
            }
            arm_motor0.setPower(0);
            arm_motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

    private void evaluateGrabberInput() {
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

    private void controlGrabber() {
        greenGrabber.setPosition(greenGrabberState.ServoPosition);
        blueGrabber.setPosition(greenGrabberState.ServoPosition);
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

    private void runCollector() {
        //Test for set points collecting
        if (a2PressedEvaluator.evaluate()) {
            collectorState = 1;
        } else if (b2PressedEvaluator.evaluate()) {
            collectorState = 2;
        } else if (x2PressedEvaluator.evaluate()) {
            collectorState = 3;
        } else if (y2PressedEvaluator.evaluate()) {
            collectorState = 4;
        } else if (rb2PressedEvaluator.evaluate()) {
            collectorState = 5;
        } else if (lb2PressedEvaluator.evaluate()) {
            collectorState = 6;
        }
        telemetry.addData("Collector state", collectorState);
        switch (collectorState) {
            case 1:
                armTargetDegrees = -21;
                extendLength = 0;
                wrist.setPosition(.45);
                break;
            case 2:
                armTargetDegrees = -26;
                extendLength = 0;
                wrist.setPosition(1);
                break;
            case 3:
                armTargetDegrees = 0;
                extendLength = 0;
                wrist.setPosition(.7);
                break;
            case 4:
                //degtarget = 170;
                //extendLength =0 ;
                wrist.setPosition(1);
                break;
            case 5:
                armTargetDegrees = 120;
                extendLength = 18;
                wrist.setPosition(1);
                break;
            case 6:
                armTargetDegrees = 153;
                extendLength =0 ;
                wrist.setPosition(.9);
                break;


        }
        telemetry.addData("Wrist pos:",wrist.getPosition());
        greenGrabber.setPosition(greenGrabberState.ServoPosition);
        blueGrabber.setPosition(blueGrabberState.ServoPosition);
    }

    private void runTelemetry() {
        telemetry.addData("f", f);
        //telemetry.addData("ff", ff);
        //telemetry.addData("Armpower", armpower);
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
}


