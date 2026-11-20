package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Turret {
    private final double maxTurretRot = 350;

    private Servo rotationServo;
    private DcMotorEx shooterMotor;

    public Turret(HardwareMap hardwareMap, String rotationName, String shooterName) {
        rotationServo = hardwareMap.get(Servo.class, rotationName);
        shooterMotor = hardwareMap.get(DcMotorEx.class, shooterName);

        MotorConfigurationType motorType = shooterMotor.getMotorType().clone();

        motorType.setTicksPerRev(103.8);
        motorType.setMaxRPM(1620);

        shooterMotor.setMotorType(motorType);
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

    public double getShooterRPM() {
//        return (shooterMotor.getVelocity(AngleUnit.DEGREES) * (6000/1620.0) * (14 / 8.0) * 60.0 * (1 / 360.0));
        return shooterMotor.getVelocity(AngleUnit.DEGREES)
                * (60.0 / 360.0)
                * (14.0 / 8.0);
    }

//    public double getShooterRPM() {
//        double motorRPM = shooterMotor.getVelocity(AngleUnit.DEGREES) * (60.0 / 360.0);
//
////        double gearRatio = 14.0 / 8.0;
//        double gearRatio = 1;
//
//        return motorRPM * gearRatio;
//    }
}
