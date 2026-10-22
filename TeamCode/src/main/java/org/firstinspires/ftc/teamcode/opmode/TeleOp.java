package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystem.TestSubsystem;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "Starter Bot")
public class TeleOp extends OpMode {
    Robot robot;
    final double DEADBAND = 0.05;
    TestSubsystem subsys;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);
        subsys = new TestSubsystem(hardwareMap);
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
        telemetry.addData("fl", fl);
        telemetry.addData("fr", fr);
        telemetry.addData("bl", bl);
        telemetry.addData("fr", br);

        double power = applyDeadband(gamepad1.right_trigger - gamepad1.left_trigger);

//        robot.collector.setCenterPower(power);
//        robot.collector.setSidePower(power);

//        robot.collector.setSidePower(1);

        subsys.setPower(power);
    }

    private double applyDeadband(double value) {
        return Math.abs(value) < DEADBAND ? 0.0 : Range.clip(value, -1.0, 1.0);
    }
}
