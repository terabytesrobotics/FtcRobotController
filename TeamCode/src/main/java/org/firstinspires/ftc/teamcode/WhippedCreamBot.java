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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
@TeleOp (name = "Whipped Cream")
public class WhippedCreamBot extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();


    private DcMotorEx WhipMotor;
    Double WhipPower = 1.0;
    Double revpermin = 435.0;
Double WhipSpeed=103.6*revpermin;




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


        WhipMotor = hardwareMap.get(DcMotorEx.class,"Whipper");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        WhipMotor.setDirection(DcMotorEx.Direction.FORWARD);


        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

    if (opModeIsActive()) {

        while (opModeIsActive()) {

WhipMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //WhipMotor.setPower(WhipPower);
            WhipMotor.setVelocity(WhipSpeed);


            telemetry.addData("WhipPower",WhipPower);
            telemetry.addData("WhipSpeed", (WhipMotor.getVelocity()/-3000));
            telemetry.addData("WhipCurrent", WhipMotor.getCurrent (CurrentUnit.MILLIAMPS));
            telemetry.update();

          }


        }
    }

}
