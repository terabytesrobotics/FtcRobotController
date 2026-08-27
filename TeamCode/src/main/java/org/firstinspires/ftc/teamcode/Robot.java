package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystem.Collector;
import org.firstinspires.ftc.teamcode.subsystem.Drive;

public class Robot {
    GoBildaPinpointDriver pinpoint;
    public Drive drive;
    public Collector collector;
    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    private final int moveToThreshold = 30;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        drive = new Drive(hardwareMap);
        collector = new Collector(hardwareMap);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // configure pinpoint
        pinpoint.setOffsets(-100.0, -30.0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();

        // set the starting location
        pinpoint.setPosition(new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0));

        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    public void update() {
        pinpoint.update();
    }

    public Pose2D getPos() {
        return pinpoint.getPosition();
    }

    public double getX() {
        return pinpoint.getPosX(DistanceUnit.MM);
    }

    public double getY() {
        return pinpoint.getPosY(DistanceUnit.MM);
    }

    public double getHeading() {
        return pinpoint.getHeading(AngleUnit.DEGREES);
    }

    public double getHeadingRad() {
        return pinpoint.getHeading(AngleUnit.RADIANS);
    }

    /**
    * @return state of completion (with accuracy of moveToThreshold in millimeters
    * */
    public boolean moveTo(double x, double y) {
        Pose2D pos = pinpoint.getPosition();

        double currentX = pos.getX(DistanceUnit.MM);
        double currentY = pos.getY(DistanceUnit.MM);

//        double frontLeftPower = forward + strafe + rotate;
//        double frontRightPower = forward - strafe - rotate;
//        double backLeftPower = forward - strafe + rotate;
//        double backRightPower = forward + strafe - rotate;

        double dX = x - currentX;
        double dY = y - currentY;

        double dist = Math.hypot(dX, dY);

        if (dist < moveToThreshold) {
            drive.setDrivePowers(0, 0, 0, 0);
            return true;
        }

        double strafe = -dY / dist;
        double forward = dX / dist;

        // temporary decel
        if (Math.abs(dX) < 150) {
            forward = 1 / dX;
        }
        if (Math.abs(dY) < 150) {
            strafe = 1 / dY;
        }

        telemetry.addData("delta x", dX);
        telemetry.addData("delta y", dY);
        telemetry.addData("strafe", strafe);
        telemetry.addData("forward", forward);

        double fl = forward + strafe;
        double fr = forward - strafe;
        double bl = forward - strafe;
        double br = forward + strafe;

        // denominator
        double d = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br))));

        if (Math.abs(dX) < moveToThreshold && Math.abs(dY) < moveToThreshold) {
            return true;
        }

        telemetry.addData("fl", fl);
        telemetry.addData("fr", fr);
        telemetry.addData("bl", bl);
        telemetry.addData("br", br);

        drive.setDrivePowers(fl / d,fr / d, bl / d, br / d);
//        drive.setDrivePowers(fl / 5,fr / 5, bl / 5, br / 5);

        return false;
    }
}
