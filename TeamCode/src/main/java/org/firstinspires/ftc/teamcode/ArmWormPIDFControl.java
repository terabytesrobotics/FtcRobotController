package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp (name = "Worm Arm Control PID plus F")
public class ArmWormPIDFControl extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private PIDController armcontrol;

    public static double p = 0.005, i = 0.005, d = 0.0002;
    public static double f = 0.0;

   // Angle below horiontal at start in degrees.  horizontal is 0.
    public static double angleoffset = -28;

    //Max angle limit
    public static double maxangle = 135
            ;

    //Max arm extension in cm
    public static double maxextend = 20;
    public static int extendTolerance = 4;

    public static double extendPower = 0.8

            ;

    //Length to extend in cm
    public static double extendLength = 0;

    public static int extendTicTarget =0;

    public double fmax = 0;

    public static double extendergearratio = (5.2);


    static double extender_tics_per_cm = extendergearratio*28/0.8;

    public static double degtarget = 0;
    public static double target = 0.0;

    public static double armTolerance = 10;

    private final double gear_ratio = 13.7;

    private static double worm_ratio =28;

    private final double arm_ticks_per_degree = worm_ratio*28*gear_ratio/360;

    private DcMotorEx arm_motor0;
    //private DcMotorEx arm_motor1;

    private DcMotorEx extender;


    /**
     * Override this method and place your code here.
     * <p>
     * Please do not catch {@link InterruptedException}s that are thrown in your OpMode
     * unless you are doing it to perform some brief cleanup, in which case you must exit
     * immediately afterward. Once the OpMode has been told to stop, your ability to
     * control hardware will be limited.
     *
     * @throws InterruptedException When the OpMode is stopped while calling a method
     *                              that can throw {@link InterruptedException}
     */
    @Override
    public void runOpMode() throws InterruptedException {

        armcontrol = new PIDController(p,i,d);
        arm_motor0 = hardwareMap.get(DcMotorEx.class,"arm_motor0");
        //arm_motor1 = hardwareMap.get(DcMotorEx.class,"arm_motor1");
        extender = hardwareMap.get(DcMotorEx.class,"extender");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm_motor0.setDirection(DcMotorEx.Direction.FORWARD);
        //arm_motor1.setDirection(DcMotorEx.Direction.FORWARD);
        extender.setDirection(DcMotorSimple.Direction.REVERSE);



        extender.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        extender.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        extender.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        //extender.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        extender.setTargetPosition(0);


        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

    if (opModeIsActive()) {

        while (opModeIsActive()) {



            target = ((Math.min(degtarget,maxangle))-angleoffset)*arm_ticks_per_degree;

            armcontrol.setPID(p,i,d);

            armcontrol.setTolerance(armTolerance);


            int armPos = arm_motor0.getCurrentPosition();
            int extendPos = extender.getCurrentPosition();

            double pid = armcontrol.calculate(armPos,target);

            double ff = ((f)+(fmax*extendPos/(maxextend*extender_tics_per_cm)))*Math.cos(Math.toRadians(degtarget));



            double armpower = pid+ff;
            //Set extension
            extendTicTarget = (int) ((Math.min(extendLength,maxextend))*extender_tics_per_cm);
            extender.setTargetPosition(extendTicTarget);
            extender.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);


            arm_motor0.setPower(armpower);
            //arm_motor1.setPower(armpower);
            extender.setPower(extendPower);

            telemetry.addData("f",f);
            telemetry.addData("ff",ff);
            telemetry.addData("Armpower",armpower);
            telemetry.addData("Position", armPos);
            telemetry.addData("target",target);
            telemetry.addData("AnglePos", (armPos/arm_ticks_per_degree)+angleoffset);
            telemetry.addData("angle target",degtarget);
            telemetry.addData("Extender Tic Target", extendTicTarget);
            telemetry.addData("Extender Current Tics",extendPos);
            telemetry.addData("Extender Target Length cm", extendLength);
            telemetry.addData("Extend Current Length", extendPos/extender_tics_per_cm);
            telemetry.addData("extender power",extender.getPower());

            telemetry.update();

          }


        }
    }

}
