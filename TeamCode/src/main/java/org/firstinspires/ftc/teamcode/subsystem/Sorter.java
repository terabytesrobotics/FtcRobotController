package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Sorter {
    private CRServo pollenKicker;
    private DcMotorEx nectarKicker;

    public Sorter(HardwareMap hardwareMap) {
        pollenKicker = hardwareMap.get(CRServo.class, "pollenKicker");
        nectarKicker = hardwareMap.get(DcMotorEx.class, "nectarKicker");
    }

    public void setPollenKickerPower(double power) {
        pollenKicker.setPower(power);
    }

    public void setNectarKickerPower(double power) {
        nectarKicker.setPower(power);
    }
}
