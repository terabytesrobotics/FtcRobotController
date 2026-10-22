package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TestSubsystem {
    private static final String NAME = "testMotor";

    private DcMotorEx motor;
    private CRServo sideIntake;
//    private CRServo rightIntake;

    public TestSubsystem(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, NAME);
//        rightIntake = hardwareMap.get(CRServo.class, RIGHT_INTAKE_NAME);
    }

    public void setPower(double power) {
        motor.setPower(power);
    }
}
