package org.firstinspires.ftc.teamcode;


import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.apache.commons.math3.stat.descriptive.summary.Sum;
import org.ejml.equation.IntegerSequence;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
    @Config
    @TeleOp
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

        /*Pixel, tape, and tile colors
        Purple RGB 170, 150, 222; Hex #aa96de; Hue 257
        Green RGB 102, 190, 63; Hex #66be3f; Hue 102
        Yellow RGB 247, 192, 1; Hex #f7c001; Hue 47
        White RGB 242, 240, 253; Hex #f2f0fd; Hue any, Sat small, Value high V>98 & S<2
        Red RGB 211, 0, 11; Hex #d3000b; Hue 357
        Blue RGB 0, 113, 189; Hex #0071bd; Hue 204
        Gray (tile) V<80, S<2
         */
       /* public enum ColorTrigger {
            White, Gray, Blue, Red, Green, Yellow, Purple
        }
*/
        @Override
        public void runOpMode() {
            // Get the color sensor from hardwareMap
            Color1 = hardwareMap.get(RevColorSensorV3.class,"color1");




            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            // Wait for the Play button to be pressed
            waitForStart();

            // While the Op Mode is running, update the telemetry values.
            while (opModeIsActive()) {

                //Color.RGBToHSV(Color1.red(), Color1.green(), Color1.blue(), HSV);
                Red = Color1.red();
                Blue = Color1.blue();
                Green = Color1.green();
                Alpha = Color1.alpha();
                Prox = Color1.getDistance(DistanceUnit.CM);
                Argb = Color1.argb();

                telemetry.addData("Red", Red);
                telemetry.addData("Green", Green);
                telemetry.addData("Blue", Blue);
                telemetry.addData("alpha", Alpha);
                telemetry.addData("prox", Prox);
                telemetry.addData("argb", Argb);
                /*telemetry.addData("Hue", HSV[0]);
                telemetry.addData("Sat", HSV[1]);
                telemetry.addData("Val", HSV[2]);*/
                telemetry.addData("Detected Color", ColorDetect());


                telemetry.update();
            }
        }



    public String ColorDetect() {
        int SumRed = 0;
        int SumGreen = 0;
        int SumBlue = 0;
        int polls = 20; //number of readings to average


        for (int i = 0; i < polls; i++) {
            SumRed += Color1.red();
            SumBlue += Color1.blue();
            SumGreen += Color1.green();

            // Delay between polls (adjust as needed)
            sleep(5);
        }
        // Calculate the average values
        int avgRed = SumRed / polls;
        int avgBlue = SumBlue / polls;
        int avgGreen = SumGreen / polls;

        // Convert RGB to HSV
        float[] hsvValues = new float[3];
        android.graphics.Color.RGBToHSV(avgRed, avgGreen, avgBlue, hsvValues);

        float Hue = hsvValues[0];
        float Sat = hsvValues[1];
        float Val = hsvValues[2];

        telemetry.addData("Hue",hsvValues[0]);
        telemetry.addData("Sat",hsvValues[1]);
        telemetry.addData("Val",hsvValues[2]);

                /*Pixel, tape, and tile colors
        Purple RGB 170, 150, 222; Hex #aa96de; Hue 257
        Green RGB 102, 190, 63; Hex #66be3f; Hue 102
        Yellow RGB 247, 192, 1; Hex #f7c001; Hue 47
        White RGB 242, 240, 253; Hex #f2f0fd; Hue any, Sat small, Value high V>98 & S<2
        Red RGB 211, 0, 11; Hex #d3000b; Hue 357
        Blue RGB 0, 113, 189; Hex #0071bd; Hue 204
        Gray (tile) V<80, S<2
         */

        if (Sat < .4 && Val > 10) {
                return "White";
        } else if ((Sat < .4 && Val < 4)){
                return "Gray";
        } else if (Hue < 40 || Hue > 345) {
            return "Red";
        } else if (Hue >= 214 && Hue <= 230) {
            return "Blue";
        } else if (Hue >= 200 && Hue <= 213) {
            return "Purple";
        } else if (Hue >= 104 && Hue <= 144) {
            return "Green";
        } else if (Hue >= 50 && Hue <= 89) {
            return "Yellow";
        } else {
            return "Unknown";
        }


    }


    }




