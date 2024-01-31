package org.firstinspires.ftc.teamcode;


import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
    @Config
    public class TestColorSensor_LAK extends LinearOpMode {
        // Define a variable for our color sensor
        RevColorSensorV3 Color1;
        public static double Red;
        public static double Blue;
        public static double Green;

        public static double Alpha;

        public static double Prox;

        public static double Argb;

        public static String colorToString;

        public float[] HSV;

        /*public void Enum {
            Red = Color.RED;
            Blue = Color.BLUE;
        }*/

        /*Pixel colors
        Purple RGB 170, 150, 222; Hex #aa96de; Hue 257
        Green RGB 102, 190, 63; Hex #66be3f; Hue 102
        Yellow RGB 247, 192, 1; Hex #f7c001; Hue 47
        White RGB 242, 240, 253; Hex #f2f0fd; Hue 249
        Red RGB 211, 0, 11; Hex #d3000b; Hue 357
        Blue RGB 0, 113, 189; Hex #0071bd; Hue 204
         */

        @Override
        public void runOpMode() {
            // Get the color sensor from hardwareMap




            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            // Wait for the Play button to be pressed
            waitForStart();

            // While the Op Mode is running, update the telemetry values.
            while (opModeIsActive()) {

                Color.RGBToHSV(Color1.red(),Color1.green(),Color1.blue(),HSV);
                Red = Color1.red();
                Blue = Color1.blue();
                Green = Color1.green();
                Alpha = Color1.alpha();
                Prox = Color1.getDistance(DistanceUnit.CM);
                Argb = Color1.argb();

                telemetry.addData("Red",Red);
                telemetry.addData("Green", Green);
                telemetry.addData("Blue", Blue);
                telemetry.addData("alpha",Alpha);
                telemetry.addData("prox",Prox);
                telemetry.addData("argb",Argb);
                telemetry.addData("Hue" ,HSV[0]);
                telemetry.addData("Sat",HSV[1]);
                telemetry.addData("Val",HSV[2]);



                telemetry.update();
            }
        }
    }


