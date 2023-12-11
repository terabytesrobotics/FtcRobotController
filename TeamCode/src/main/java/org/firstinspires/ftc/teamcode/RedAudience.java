package org.firstinspires.ftc.teamcode;

import android.icu.text.CaseMap;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.Processors.WindowBoxesVisionProcessor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Config
@Autonomous(group = "Competition")
public class RedAudience extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;
    /**
     * The variable to store our instance of the prop finding vision processor.
     */private WindowBoxesVisionProcessor propfinder;
    public CameraCalibration calibration;

    /**
     * height and width of cam used for prop "Cam2"
     */
    int HeightCam2 = 480;
    int WidthCam2 = 640;

    /**
     * Rows and columns for prop detection grid
     */
    public static int rows = 3;
    public static int cols = 1;

    public static String color = "RED";//Change to BLUE as necessary.

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    String propLocation;

    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;
    /**Blinkin Patterns using red, blue, or green
     * BLUE
     * RED
     * GREEN
     * BREATH_BLUE
     * BREATH_RED
     * DARK_BLUE
     * DARK_RED
     * DARK_GREEN
     * HEARTBEAT_BLUE
     * HEARTBEAT_RED
     * SHOT_BLUE
     * SHOT_RED
     * LIGHT_CHASE_BLUE
     * LIGHT_CHASE_RED
     * STROBE_BLUE
     * STROBE_RED
     *

    */

    @Override
    public void runOpMode() throws InterruptedException {

        //Sets up vision processor
        initpropfinder();

        /**
         * Add Blinken code to id alliance and position
         */


        // Wait for the DS start button to be touched.
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (color=="RED") {
            Object redresults[] = propfinder.topbox(WidthCam2, HeightCam2, rows, cols, "RED");
            if (redresults[1] instanceof Integer && (int) redresults[1] < 1 ) {
            propLocation = "left";
            }
            else if (redresults[1] instanceof Integer && (int) redresults[1] == 1) {
                propLocation = "center";
            }
            else if (redresults[1] instanceof Integer && (int) redresults[1] > 1) {
                propLocation = "right";
            }

           //Default = center
            else propLocation = "center";

            telemetry.addLine("Red Row " + redresults[0]);
            telemetry.addLine("Red Col" + redresults[1]);
            telemetry.addLine("Red value " + redresults[2]);


        }

        else if (color=="BLUE") {
            Object blueresults[] = propfinder.topbox(WidthCam2, HeightCam2, rows, cols, "BLUE");
            if (blueresults[1] instanceof Integer && (int) blueresults[1] < 1 ) {
                propLocation = "left";
            }
            else if (blueresults[1] instanceof Integer && (int) blueresults[1] == 1) {
                propLocation = "center";
            }
            else if (blueresults[1] instanceof Integer && (int) blueresults[1] > 1) {
                propLocation = "right";
            }
            else propLocation = "center";


            telemetry.addLine("Blue Row " + blueresults[0]);
            telemetry.addLine("Blue Col" + blueresults[1]);
            telemetry.addLine("Blue value " + blueresults[2]);

        }
        else return;
        /**Turn off front camera
         * Turn on rear camera and set up Apriltag processor
         */

        /**
         * Add drive to pose to drop first pixel based on propLocation left, center, right
         * Drop pixel
         * If on audience side, drive to backstage to score deployment position
         * Move arm and wrist to score position
         * Drive to April tag detect position
         * Do Apriltag alignment based on propLocation
         * Drop pixel
         */



        telemetry.update();


       /* SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        waitForStart();
        if (isStopRequested()) return;

        Trajectory traj = drive
                .trajectoryBuilder(new Pose2d())
                .build();
        drive.followTrajectory(traj);

        */
    }

    private void initpropfinder() {

        propfinder = new WindowBoxesVisionProcessor();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), propfinder);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, propfinder);
        }

    }   // end method initpropfinder()
}
