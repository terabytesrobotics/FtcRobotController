package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Turret {
    private final double maxTurretRot = 350;

    private Servo rotationServo;
    private DcMotorEx shooterMotor;

    public Turret(HardwareMap hardwareMap, String rotationName, String shooterName) {
        rotationServo = hardwareMap.get(Servo.class, rotationName);
        shooterMotor = hardwareMap.get(DcMotorEx.class, shooterName);
    }

    // servo to turret is 1:1.16
    public boolean rotateTo(double deg, Telemetry t) {
        deg = Math.max(-203, Math.min(deg, 203));
        double servoDeg = deg / 1.16;
        double pwm = (servoDeg + maxTurretRot / 2) / maxTurretRot;
        pwm = Math.max(0.05, Math.min(pwm, 0.95));
        rotationServo.setPosition(pwm);

        t.addData("target turret angle", deg);
        t.addData("servo angle", servoDeg);
        t.addData("pwm", pwm);

        return true;
    }

    public boolean setShooterPower(double power) {
        shooterMotor.setPower(power);

        return true;
    }
}
