package org.firstinspires.ftc.teamcode.Processors;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class SampleDetectVisionProcessor implements VisionProcessor {
    public Mat lastFrame;
  /* public Mat boxmaxes;
    public int Rows;
    public int Cols;
    public int colorindex=0;
    public int Height;
    public int Width;
    public Rect maxroi;

    public Rect startrect;*/

    public Paint rectPaint;


    @Override
    public void init(int width, int height, CameraCalibration calibration) {

     /*   startrect=new Rect(0,0,width/2,height/2);
        maxroi=new Rect(0,0,width/2,height/2);*/



    }


    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        lastFrame = frame.clone();

        //return lastFrame;


        // Convert to HSV color space
        Mat hsvFrame = new Mat();
        Imgproc.cvtColor(lastFrame, hsvFrame, Imgproc.COLOR_RGB2HSV);

        // Detect colors (red, blue, and yellow as an example)
        Mat redMask = new Mat();
        Mat blueMask = new Mat();
        Mat yellowMask = new Mat();

        // Define the lower and upper boundaries for red and blue in HSV space
        Scalar lowerRed = new Scalar(0, 70, 50);
        Scalar upperRed = new Scalar(10, 255, 255);
        Scalar lowerBlue = new Scalar(100, 150, 0);
        Scalar upperBlue = new Scalar(140, 255, 255);
        Scalar lowerYellow = new Scalar(20, 100, 100);
        Scalar upperYellow = new Scalar(30, 255, 255);

        // Detect red
        Core.inRange(hsvFrame, lowerRed, upperRed, redMask);

        // Detect blue
        Core.inRange(hsvFrame, lowerBlue, upperBlue, blueMask);

        // Detect yellow
        Core.inRange(hsvFrame, lowerYellow, upperYellow, yellowMask);

        // Combine masks for red, blue, and yellow detection
        Mat combinedMask = new Mat();
        Core.addWeighted(redMask, 1.0, blueMask, 1.0, 0.0, combinedMask);
        Core.addWeighted(combinedMask, 1.0, yellowMask, 1.0, 0.0, combinedMask);

        // Find contours (which helps detect shapes)
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(combinedMask, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        // Process detected contours (shapes)
        for (MatOfPoint contour : contours) {
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());

            // Approximate contour to polygon to determine the shape
            double peri = Imgproc.arcLength(contour2f, true);
            MatOfPoint2f approx = new MatOfPoint2f();
            Imgproc.approxPolyDP(contour2f, approx, 0.04 * peri, true);
            int numVertices = (int) approx.total();

            // Only process rectangles (4 vertices)
            if (numVertices == 4) {
                // Get bounding box around the shape
                Rect boundingRect = Imgproc.boundingRect(new MatOfPoint(approx.toArray()));

           /* // Identify the shape based on the number of vertices
            String shape = "";
            if (numVertices == 3) {
                shape = "Triangle";
            } else if (numVertices == 4) {
                shape = "Rectangle";
            } else {
                shape = "Circle";
            }*/

                // Draw bounding box around the detected shape
                Imgproc.rectangle(lastFrame, boundingRect, new Scalar(0, 255, 0), 2);

                // Put text on the detected object to indicate color and shape
                Imgproc.putText(lastFrame, "Rectangle", new Point(boundingRect.x, boundingRect.y - 10),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255), 2);

            }

        }
        return lastFrame; // Return the processed frame with shapes and bounding boxes
    }


    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // Drawing is handled in processFrame now, nothing needed here.
        // Convert the lastFrame Mat to a Bitmap
        Bitmap bmp = Bitmap.createBitmap(lastFrame.cols(), lastFrame.rows(), Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(lastFrame, bmp);

        // Draw the Bitmap on the Canvas
        canvas.drawBitmap(bmp, 0, 0, null);

        /*if (lastFrame != null) {
            Bitmap bmp = Bitmap.createBitmap(lastFrame.cols(), lastFrame.rows(), Bitmap.Config.ARGB_8888);
            Utils.matToBitmap(lastFrame, bmp);

            // Draw the Bitmap on the Canvas
            canvas.drawBitmap(bmp, 0, 0, null);
        }

        // Draw the detected rectangle (if applicable)
        if (maxroi != null) {
            rectPaint = new Paint();
            rectPaint.setStyle(Paint.Style.STROKE);
            rectPaint.setStrokeWidth(scaleCanvasDensity * 10);
            rectPaint.setColor(Color.RED); // For example, you can set the color dynamically
            canvas.drawRect(makeGraphicsRect(maxroi, scaleBmpPxToCanvasPx), rectPaint);
        }
    }*/
    }
}

  /*  @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {


            rectPaint = new Paint();
           if (colorindex == 1) {
                rectPaint.setColor(Color.RED);
            } else if (colorindex == 2) {
                rectPaint.setColor(Color.BLUE);
            } else return;


            rectPaint.setStyle(Paint.Style.STROKE);
            rectPaint.setStrokeWidth(scaleCanvasDensity * 10);

        if(maxroi!=null ) {

         canvas.drawRect(makeGraphicsRect(maxroi, scaleBmpPxToCanvasPx), rectPaint);
        }
        else {

          canvas.drawRect(makeGraphicsRect(startrect,scaleBmpPxToCanvasPx), rectPaint);
             }

    }
*/

// NOTE: This is very hacked to only check the second row of 3.
 /*   public Object[] topbox(int width,int height, int rows, int cols, AllianceColor color) {
        if (lastFrame == null) {
            return new Object[]{};
        }
        // Comment out this is RGB Mat evalFrame = lastFrame.clone();
        //This converts to YCRCB
        Mat ycrcbEvalFrame = new Mat();
        Imgproc.cvtColor(lastFrame, ycrcbEvalFrame,Imgproc.COLOR_RGB2YCrCb);
        Mat boxes[][] = new Mat[1][cols];
        double crvals[][] = new double[rows][cols];
        double cbvals[][] = new double[rows][cols];
        double yvals[][] = new double[rows][cols];

        double avred;
        double avy;
        double rcor;
        rcor = 0;



        boxmaxes = new Mat(rows, cols, CvType.CV_64FC1);

        Width = width;
        Height = height;
        Rows = rows;
        Cols = cols;
        int maxrow = -1;
        int maxcol = -1;
        double maxval = -1;


        if (color == AllianceColor.RED) {
            //index 2 for RGB, 1 for ycrcb
            colorindex = 1;


        } else if (color == AllianceColor.BLUE) {
            //index 0 for RGB, 2 for ycrcb

            colorindex = 2;
        } else {
            System.err.println("Error: Color not recognized.");
        }
        // Create submats and store them in the boxes array
        int boxWidth = Width / Cols;
        int boxHeight = Height / Rows;

        // TODO: THIS IS HACKED
        for (int r = 1; r < 2; r++) {
            //boxmaxes = null;
            for (int c = 0; c < Cols; c++) {

                Rect roi = new Rect(c * boxWidth, r * boxHeight, boxWidth, boxHeight);
                boxes[0][c] = new Mat(ycrcbEvalFrame, roi);

                // Calculate the average selected color intensity in the box
                Scalar avgColor = Core.mean(boxes[0][c]);


                if (color == AllianceColor.RED) {

                    boxmaxes.put(r, c, avgColor.val[1]);

                } else if (color == AllianceColor.BLUE) {

                    boxmaxes.put(r, c, avgColor.val[2]);

                }
            }


        }
        Core.MinMaxLocResult result = Core.minMaxLoc(boxmaxes);

        maxval = result.maxVal;
        Point maxLoc = result.maxLoc;
        maxroi = new Rect((int) maxLoc.x * boxWidth, (int)maxLoc.y * boxHeight, boxWidth, boxHeight);

        maxrow = (int) maxLoc.y;
        maxcol = (int) maxLoc.x;
        return new Object[]{maxrow, maxcol, maxval};

    }
    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        if(rect !=null){
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);}
        else {return null;
        }


        }

    }*/

