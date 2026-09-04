package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystem.Collector;
import org.firstinspires.ftc.teamcode.subsystem.Drive;
import org.firstinspires.ftc.teamcode.util.PIDController;

public class Robot {
    GoBildaPinpointDriver pinpoint;
    public Drive drive;
    public Collector collector;
    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    private final int moveToThreshold = 5;
    private final double MAX_DRIVE_OUTPUT_POWER = 0.95;
    public final double kP_POWER_PER_MM = 1.0 / 200; // 100% power (~torque) / 1000mm
    public final double kI_POWER_PER_MM_SEC = 0.0001; // 0% power / mm * sec
    public final double kD_POWER_PER_MM_PER_SEC = 0.0; // 0% power / (mm/sec)
    public final PIDController xDriveController = new PIDController(kP_POWER_PER_MM, kI_POWER_PER_MM_SEC, kD_POWER_PER_MM_PER_SEC);
    public final PIDController yDriveController = new PIDController(kP_POWER_PER_MM, kI_POWER_PER_MM_SEC, kD_POWER_PER_MM_PER_SEC);

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
        drive.setDrivePowers(0, 0, 0, 0);
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
    public boolean moveTo(double destX, double destY) {
        Pose2D pos = pinpoint.getPosition();

        double currentX = pos.getX(DistanceUnit.MM);
        double currentY = pos.getY(DistanceUnit.MM);

        double dX = destX - currentX;
        double dY = destY - currentY;

        double dist = Math.hypot(dX, dY);

        if (dist < moveToThreshold) {
            drive.setDrivePowers(0, 0, 0, 0);
            return true;
        }

        double strafe = yDriveController.calculate(destY, currentY);
        strafe *= -1.0;
        double forward = xDriveController.calculate(destX, currentX);

//        telemetry.addData("delta x", dX);
//        telemetry.addData("delta y", dY);
//        telemetry.addData("strafe", strafe);
//        telemetry.addData("forward", forward);

        double fl = forward + strafe;
        double fr = forward - strafe;
        double bl = forward - strafe;
        double br = forward + strafe;

        // denominator
        double d = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br))));
        //d *= 3;
        d = Math.min(0.0, d);
        d = Math.max(1.0, d);

        if (Math.abs(dX) < moveToThreshold && Math.abs(dY) < moveToThreshold) {
            return true;
        }

//        telemetry.addData("fl", fl);
//        telemetry.addData("fr", fr);
//        telemetry.addData("bl", bl);
//        telemetry.addData("br", br);

        drive.setDrivePowers(fl / d,fr / d, bl / d, br / d);
        return false;
    }
}
