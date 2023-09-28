/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.videoio.VideoCapture;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

/*
 * This OpMode illustrates the basics of AprilTag recognition and pose estimation, using
 * the easy way.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@TeleOp(name = "Red Prop2 // OpenCV", group = "Concept")
//@Disabled
public class RedPropTest2 extends LinearOpMode {

Prop_Pipeline prop_pipeline;
OpenCvInternalCamera Prop_cam;

VideoCapture prop_scan_capture;
Mat prop_scan;
boolean prop_scanned = false;



    @Override
    public void runOpMode() {

        init_propscan();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        //telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {


                // Push telemetry to the Driver Station.
                telemetry.update();

                //if (!prop_scanned) {

                    VideoCapture prop_scan_capture = new VideoCapture();


                 /*   if (!prop_scan_capture.isOpened()) {
                        //System.out.println("Error: Could not open camera.");
                        telemetry.addLine("Not captured");
                        telemetry.update();
                        sleep(10000);
                        return;
                    }*/

                       // while (true) {
                           prop_scan = new Mat();

                    prop_scanned = prop_scan_capture.read(prop_scan);


                         //   if (prop_scan_capture.read(prop_scan)) {

                                int height = prop_scan.height();
                                int width = prop_scan.width();
                                int sectionwidth = width / 3;

                                telemetry.addData("W", width);
                                telemetry.addData("H", height);
                                telemetry.addData("SW", sectionwidth);
                                telemetry.update();
                                sleep(10000);


                    double totalRed = 0.0;
                    int pixelCount = 0;
                    int[] StartX = new int[3];
                    int[] EndX = new int[3];
                    double[] sectionR = new double[3];


                    for (int i = 0; i < 3; i++) {

                        StartX[i] = i * sectionwidth;
                        EndX[i] = (1 + i) * sectionwidth;
                        Rect sectionRect = new Rect(StartX[i], 0, sectionwidth, height);
                        Mat section = new Mat(prop_scan, sectionRect);

                        for (int row = 0; row < section.rows(); row++) {
                            for (int col = 0; col < section.cols(); col++) {
                                double[] pixel = section.get(row, col);
                                double blue = pixel[0];
                                double green = pixel[1];
                                double red = pixel[2];

                                // Calculate the average red value
                                totalRed += red;
                                pixelCount++;


                            }
                        }
                        sectionR[i] = totalRed;
                        telemetry.addData("Section" + i, sectionR[i]);

                        telemetry.update();
                    }


                           // } //else {
                    //System.out.println("Error: Could not read frame.");
                    //break;
                     //       }
               // }


            }


         }   // end method runOpMode()

    }
    class Prop_Pipeline extends OpenCvPipeline {

        // boolean viewportPaused = false;
        //Mat prop_scan = new Mat();
        Mat BGRa = new Mat();
        Mat R = new Mat();
        int avg;


        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Y channel to the 'Y' variable
         */
        void inputToR(Mat input) {

            ArrayList<Mat> BGRaChannels = new ArrayList<Mat>(4);
            Core.split(BGRa, BGRaChannels);
            R = BGRaChannels.get(2);

        }

        @Override
        public void init(Mat firstFrame) {
            inputToR(firstFrame);
        }

        @Override
        public Mat processFrame(Mat input) {
            inputToR(input);
            System.out.println("processing requested");
            avg = (int) Core.mean(R).val[0];
            BGRa.release(); // don't leak memory!
            R.release(); // don't leak memory!
            return input;
        }

        public int getAnalysis() {
            return avg;
        }

       /* @Override
        public Mat processFrame(Mat input) {


            return input;
        }*/
    }

private void init_propscan() {
        sleep(50);
    int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    Prop_cam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
    prop_pipeline = new Prop_Pipeline();
    Prop_cam.setPipeline(prop_pipeline);


    // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
    // out when the RC activity is in portrait. We do our actual image processing assuming
    // landscape orientation, though.
    Prop_cam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

    Prop_cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
        @Override
        public void onOpened() {
            telemetry.addData("cameraMonitorViewId", cameraMonitorViewId);
            telemetry.addData("Start Streaming", null);
           telemetry.update();

            Prop_cam.startStreaming(320, 240);

            sleep(50);

        }

        @Override
        public void onError(int errorCode) {
            telemetry.addData("error",errorCode);
            telemetry.update();
            /*
             * This will be called if the camera could not be opened
             */
        }
    });

}

}   // end class
