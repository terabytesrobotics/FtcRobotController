package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class ArmPIDFControl {

private PIDFController armcontrol;

public static double p = 0.0, i = 0.0, d = 0.0, f = 0.0;

//public static double target = 0.0;

private final double gear_ratio = 50.9;

private final double arm_ticks_per_degree = 28*gear_ratio/360;


}
