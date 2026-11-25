package org.firstinspires.ftc.teamcode.opmode.debug;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "TurretTest", group = "Debug")
public class TestTurretOp extends OpMode {
    Robot robot;
    private double targetPower = 0;
    private double kickerPower = 0;
    private double targetRot = 0;
    private boolean shooterEnabled = false;
    private boolean kickerEnabled = false;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        robot.update();

        if (gamepad1.yWasPressed()) {
            shooterEnabled = !shooterEnabled;
        }
        if (gamepad1.xWasPressed()) {
            kickerEnabled = !kickerEnabled;
        }

        robot.pollenTurret.rotateTo(targetRot, robot.telemetry);
        if (shooterEnabled) {
            robot.pollenTurret.setShooterPower(targetPower);
        } else {
            robot.pollenTurret.setShooterPower(0);
        }

        if (kickerEnabled) {
            robot.sorter.setPollenKickerPower(kickerPower);
            robot.sorter.setNectarKickerPower(kickerPower);
        } else {
            robot.sorter.setPollenKickerPower(0);
            robot.sorter.setNectarKickerPower(0);
        }

        if (gamepad1.dpadUpWasPressed()) {
            targetPower += .05;
        }
        if (gamepad1.dpadDownWasPressed()) {
            targetPower -= .05;
        }
//        if (gamepad1.dpadRightWasPressed()) {
//            targetRot += 10;
//        }
//        if (gamepad1.dpadLeftWasPressed()) {
//            targetRot -= 10;
//        }
        if (gamepad1.dpadRightWasPressed()) {
            kickerPower += .05;
        }
        if (gamepad1.dpadLeftWasPressed()) {
            kickerPower -= .05;
        }

        if (targetPower < 0) {
            targetPower = 0;
        }
        if (targetPower > 1) {
            targetPower = 1;
        }

        if (targetRot > 100) {
            targetRot = 100;
        }
        if (targetRot < -100) {
            targetRot = -100;
        }

        if (kickerPower < 0) {
            kickerPower = 0;
        }
        if (kickerPower > 1) {
            kickerPower = 1;
        }

        robot.telemetry.addData("shooter enabled", shooterEnabled);
        robot.telemetry.addData("kicker enabled", kickerEnabled);
        robot.telemetry.addData("shooter power", targetPower);
        robot.telemetry.addData("rot", targetRot);
        robot.telemetry.addData("kicker power", kickerPower);
        robot.telemetry.addData("velo", robot.pollenTurret.getShooterRPM());
        
        robot.updateDashboard();
    }
}
