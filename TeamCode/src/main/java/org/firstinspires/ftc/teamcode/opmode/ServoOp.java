package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystem.Turret;

@TeleOp(name = "Servo", group = "Debug")
public class ServoOp extends OpMode {
    Robot robot;
    Turret turret;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(
                telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(50);
        robot = new Robot(hardwareMap);
        turret = new Turret(hardwareMap);
    }

    @Override
    public void loop() {
        robot.update();

//        double r = applyDeadband(gamepad1.left_stick_x) * 350;
//        turret.rotateTo(r, telemetry);

        double tarX = 0.1;
        double tarY = 0.1;
        double dX = tarX - robot.getX();
        double dY = tarY - robot.getY();
//        double r = robot.getHeadingDeg() - Math.toDegrees(Math.asin(dX / (Math.sqrt(dX * dX + dY * dY))));
        double r = Math.toDegrees(Math.asin(dX / (Math.sqrt(dX * dX + dY * dY))));

        telemetry.addData("turret.targetDeg", r);
        telemetry.addData("target.dxMm", dX);
        telemetry.addData("target.dyMm", dY);

        turret.rotateTo(r, telemetry);
        telemetry.update();
    }

    private double applyDeadband(double value) {
        return Math.abs(value) < 0.05 ? 0.0 : Range.clip(value, -1.0, 1.0);
    }
}
