package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp

public class TeleOpNibus2000 extends LinearOpMode {
    private DcMotor leftFrontDrive   = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive  = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive    = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive   = null;  //  Used to control the right back drive wheel
    private DcMotor pivotDrive   = null;  //  Used to control the right back drive wheel

    private Servo grabberLeft = null; //  Used to control the left grabber servo

    private Servo grabberRight = null; //Used to control the right grabber servo
    @Override
    public void runOpMode() {
        //Wheel Drive Motors
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "fl2");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "fr3");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "bl0");
        rightBackDrive = hardwareMap.get(DcMotor.class, "br1");
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        //Arm Motors
        pivotDrive  = hardwareMap.get(DcMotor.class, "pd0");
        pivotDrive.setDirection(DcMotor.Direction.FORWARD);

        pivotDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotDrive.setTargetPosition(0);
        pivotDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);







        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if(gamepad1.a){
                pivotDrive.setTargetPosition(3246);
            }
            if(gamepad1.x){
                pivotDrive.setTargetPosition(1585);
            }
            if(gamepad1.y){
                pivotDrive.setTargetPosition(0);
            }
            telemetry.addData("target", pivotDrive.getTargetPosition());
            telemetry.update();
        }
    }
}

