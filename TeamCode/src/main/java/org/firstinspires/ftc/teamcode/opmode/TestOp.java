package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "PinpointTest", group = "Starter Bot")
public class TestOp extends OpMode {
    Robot robot;
    private double kP = 0, kI = 0, kD = 0;
    private double kPDelta, kIDelta, kDDelta;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        robot.update();

        robot.moveTo(300, 300);

        telemetry.addData("X coordinate", robot.getX());
        telemetry.addData("Y coordinate", robot.getY());
        telemetry.addData("Heading angle (deg)", robot.getHeading());



        if (gamepad1.dpadUpWasPressed()) {
            if (gamepad1.a) {
                kP += kPDelta;
            }
            if (gamepad1.b) {
                kI += kIDelta;
            }

            if (gamepad1.x) {
                kD += kDDelta;
            }
        }

        if (gamepad1.dpadDownWasPressed()) {
            if (gamepad1.a) {
                kP -= kPDelta;
            }
            if (gamepad1.b) {
                kI -= kIDelta;
            }

            if (gamepad1.x) {
                kD -= kDDelta;
            }
        }

        robot.driveTrainController.kP = kP;
        robot.driveTrainController.kI = kI;
        robot.driveTrainController.kD = kD;
    }
}
