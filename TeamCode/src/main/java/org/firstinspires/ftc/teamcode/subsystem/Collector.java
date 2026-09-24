package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Collector {
    private static final String CENTER_COLLECTOR_NAME = "center_collector";
    private static final String LEFT_INTAKE_NAME = "left_intake";
    private static final String RIGHT_INTAKE_NAME = "right_intake";

    private DcMotorEx centerCollector;
    private CRServo leftIntake;
    private CRServo rightIntake;

    public Collector(HardwareMap hardwareMap) {
        centerCollector = hardwareMap.get(DcMotorEx.class, CENTER_COLLECTOR_NAME);
        leftIntake = hardwareMap.get(CRServo.class, LEFT_INTAKE_NAME);
        rightIntake = hardwareMap.get(CRServo.class, RIGHT_INTAKE_NAME);

        centerCollector.setDirection(DcMotorSimple.Direction.FORWARD);
        centerCollector.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        centerCollector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        setPower(0.0);
    }

    public void setCenterPower(double power) {
        centerCollector.setPower(power);
    }

    public void setSidePower(double power) {
        leftIntake.setPower(power);
        rightIntake.setPower(power);
    }

    public void setPower(double power) {
        setCenterPower(power);
        setSidePower(power);
    }
}
