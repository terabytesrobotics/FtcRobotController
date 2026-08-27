package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "Starter Bot")
public class TeleOp extends OpMode {
    Robot robot;
    final double DEADBAND = 0.05;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        robot.update();

        double forward = applyDeadband(-gamepad1.left_stick_y);
        double strafe = applyDeadband(gamepad1.left_stick_x);
        double rotate = applyDeadband(gamepad1.right_stick_x);

        double fl = forward + strafe + rotate;
        double fr = forward - strafe - rotate;
        double bl = forward - strafe + rotate;
        double br = forward + strafe - rotate;

        double d = Math.max(
                Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1.0);
        double speed = 1;

        fl = speed * fl / d;
        fr = speed * fr / d;
        bl = speed * bl / d;
        br = speed * br / d;

        robot.drive.setDrivePowers(fl, fr, bl, br);

        double collectorPower = applyDeadband(gamepad1.right_trigger - gamepad1.left_trigger);

        robot.collector.setCenterPower(collectorPower);
        robot.collector.setSidePower(collectorPower);
    }

    private double applyDeadband(double value) {
        return Math.abs(value) < DEADBAND ? 0.0 : Range.clip(value, -1.0, 1.0);
    }
}
