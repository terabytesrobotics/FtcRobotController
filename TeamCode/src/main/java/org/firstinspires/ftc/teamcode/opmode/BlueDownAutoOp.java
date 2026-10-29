package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "Blue Down")
public class BlueDownAutoOp extends OpMode {
    Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);

        robot.setPos(new Pose2D(DistanceUnit.INCH, DistanceUnit.mmPerInch * 9, 76 - 7, AngleUnit.DEGREES, 180));
    }

    @Override
    public void loop() {
        robot.update();


    }
}
