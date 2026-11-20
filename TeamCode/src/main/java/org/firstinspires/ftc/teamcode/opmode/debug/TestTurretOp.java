package org.firstinspires.ftc.teamcode.opmode.debug;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "TurretTest", group = "Debug")
public class TestTurretOp extends OpMode {
    Robot robot;
    private double targetPower = 0;
    private double targetRot = 0;
    private boolean enabled = false;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        robot.update();

        if (gamepad1.yWasPressed()) {
            enabled = !enabled;
        }

        robot.pollenTurret.rotateTo(targetRot, robot.telemetry);
        if (enabled) {
            robot.pollenTurret.setShooterPower(targetPower);
        } else {
            robot.pollenTurret.setShooterPower(0);
        }

        if (gamepad1.dpadUpWasPressed()) {
            targetPower += .1;
        }
        if (gamepad1.dpadDownWasPressed()) {
            targetPower -= .1;
        }
        if (gamepad1.dpadRightWasPressed()) {
            targetRot += 10;
        }
        if (gamepad1.dpadLeftWasPressed()) {
            targetRot -= 10;
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

        robot.telemetry.addData("shooter power", targetPower);
        robot.telemetry.addData("rot", targetRot);
        robot.telemetry.addData("velo", robot.pollenTurret.getShooterRPM());
        
        robot.updateDashboard();
    }
}
