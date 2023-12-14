package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;


@Disabled
    @TeleOp(name="Test: Gordon", group="Linear OpMode")

    public class GordonsTest extends LinearOpMode {

        // Declare OpMode members for each of the 4 motors.
        private ElapsedTime runtime = new ElapsedTime();
        private DcMotor leftFrontDrive = null;
        private DcMotor leftBackDrive = null;
        private DcMotor rightFrontDrive = null;
        private DcMotor rightBackDrive = null;

        @Override
        public void runOpMode() {
            leftFrontDrive = hardwareMap.dcMotor.get("LF");
            leftBackDrive = hardwareMap.dcMotor.get("LR");
            rightFrontDrive = hardwareMap.dcMotor.get("RF");
            rightBackDrive = hardwareMap.dcMotor.get("RR");

            leftFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
            leftBackDrive.setDirection(DcMotorEx.Direction.REVERSE);
            rightFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
            rightBackDrive.setDirection(DcMotorEx.Direction.FORWARD);


            telemetry.addData("Status", "Initialized");
            telemetry.update();

            waitForStart();
            runtime.reset();

            leftFrontDrive.setPower(.5);
            rightFrontDrive.setPower(.5);

        }
    }