package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;


@SuppressWarnings("ALL")
@TeleOp(name="LK Teleop Test", group ="Real")
@Disabled
public class LKTeleopTest extends OpMode {

    private static final int secondsPermissionTimeout = Integer.MAX_VALUE;
    private static final String TAG = "Webcam Sample";

    private Click oneClick;
    private RevColorSensorV3 REVColor1;
    private RevColorSensorV3 REVColor;
    private DcMotorEx flDcMotor;
    private DcMotor frDcMotor;
    private DcMotor blDcMotor;
    private DcMotor brDcMotor;
    private DcMotorEx Shooter;
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

    //* Added by LAK
    private Writer DLog;
    private ElapsedTime runtime = new ElapsedTime();
    private PIDFCoefficients pid;

    @Override
    public void init() {


        flDcMotor = hardwareMap.get(DcMotorEx.class, "flMotor");
        flDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frDcMotor = hardwareMap.get(DcMotor.class, "frMotor");
        frDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        blDcMotor = hardwareMap.get(DcMotor.class, "blMotor");
        blDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brDcMotor = hardwareMap.get(DcMotor.class, "brMotor");
        brDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
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


        pid = Shooter.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);

        telemetry.addData("PID coefficients", String.valueOf(pid));

        telemetry.addData("Status", "Initialized");
        telemetry.addData(">", "Press Play to start");


        String directoryPath = Environment.getExternalStorageDirectory().getPath() + "/FIRST/";
        String file = directoryPath + "DLog.csv";
        telemetry.addData("filename", file);
        // telemetry.addData("dt", LocalDateTime.now());
        telemetry.update();


        try {
            DLog = new FileWriter(file, true);
        } catch (IOException e) {
            e.printStackTrace();
        }

       /* try {
            DLog.write("Time" + "," + "Shooter Speed\r\n");
            DLog.flush();
            DLog.close();
        } catch (IOException e) {
            e.printStackTrace();
        }

        */


    }

    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {


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



                try {
                    String line = runtime.toString()+","+ Shooter.getVelocity()+","+Shooter.getCurrent(CurrentUnit.AMPS);
                    DLog.write(line +"\r\n");
                    DLog.close();

                } catch (IOException e) {
                    e.printStackTrace();
                }




        telemetry.addData("flDcMotor", flDcMotor.getCurrentPosition());
        telemetry.addData("frDcMotor", frDcMotor.getCurrentPosition());
        telemetry.addData("blDcMotor", blDcMotor.getCurrentPosition());
        telemetry.addData("brDcMotor", brDcMotor.getCurrentPosition());


        telemetry.addData("flmotor speed", flDcMotor.getVelocity());
        telemetry.addData("shooter speed", Shooter.getVelocity());
        telemetry.addData("shooter position", Shooter.getCurrentPosition());

        double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x * -1;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        FL = -v1;
        FR = -v2;
        BL = v3;
        BR = -v4;

        if (gamepad1.left_bumper) {
            FL /= 2;
            FR /= 2;
            BL /= 2;
            BR /= 2;
        }

        flDcMotor.setPower(FL);
        frDcMotor.setPower(FR);
        blDcMotor.setPower(BL);
        brDcMotor.setPower(BR);
        telemetry.addData("fl", cLift.getCurrentPosition());


        // TODO: Factor out telemetry
        telemetry.addData("flMotor Power", flDcMotor.getPower());
        telemetry.addData("frMotor Power", frDcMotor.getPower());
        telemetry.addData("blMotor Power", blDcMotor.getPower());
        telemetry.addData("brMotor Power", blDcMotor.getPower());
        telemetry.addData("PlatformPos", Platform.getPosition());

        telemetry.addData("CollectorPos", rollerDirectionNonOverride);
        telemetry.addData("LFinger", LFinger.getPosition());
        telemetry.addData("RFinger", RFinger.getPosition());
        // Set Points


        //
        // er code
        ToggleShooter = this.gamepad2.b;
        if (SButtonlock && !ToggleShooter) {
            SButtonlock = false;
        }
        if (ToggleShooter && !SButtonlock) {
            ShooterStatus += .85;
            SButtonlock = true;
        }
        if (ShooterStatus >= .85 && !SButtonlock) {
            ShooterStatus = 0;
            SButtonlock = true;
        }
        Shooter.setPower(ShooterStatus);

        /*try {
            String directoryPath = Environment.getExternalStorageDirectory().getPath() + "/FIRST/";
            String file = directoryPath + "DLog.csv";
            DLog = new FileWriter(file, true);
            String line = runtime.toString() + "," + Shooter.getVelocity() + "," + Shooter.getCurrent(CurrentUnit.AMPS);
            DLog.write(line + "\r\n");
            DLog.flush();
            DLog.close();

        } catch (IOException e) {
            e.printStackTrace();
        }

         */


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
        if (PlatformPositionX < 0.2) {
            PlatformPositionX = 0.2;
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
        FinalRoller.setPosition(-rollerDirection + 0.5);
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
        LFinger.setPosition(.3 - FingerPos);
        TriggersOn = this.gamepad2.y;
        if (TriggersOn) {
            FingerPos = 0;
        } else {
            FingerPos = 0.26;
        }


        //* Collector Up and Down Code
        telemetry.addData("clift", cLift.getCurrentPosition());
        if (gamepad1.right_bumper && TopLimit.getValue() == 0) {
            cLift.setPower(1);
            telemetry.addLine("TopLimit.getValue() ==0");
        } else if (gamepad1.right_bumper && TopLimit.getValue() == 1) {
            cLift.setPower(0);
            telemetry.addLine("TopLimit.getValue() ==1");
        } else if (BottomLimit.getValue() == 0) {
            cLift.setPower(-1);
            telemetry.addLine("BottomLimit.getValue() ==0");
        } else {
            cLift.setPower(0);
            telemetry.addLine("none");
        }

             /*   if (tfod != null) {
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

              */

                /*try {
                    DLog.close();
                } catch (IOException e) {
                    e.printStackTrace();
                }


                 */

        telemetry.update();


    }



public void stop() {
    try {
        DLog.flush();
        DLog.close();
    } catch (IOException e) {
        e.printStackTrace();
    }
}
}




