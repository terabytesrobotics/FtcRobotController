package org.firstinspires.ftc.teamcode.Processors;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.view.SurfaceView;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.lang.reflect.Array;
import java.security.PublicKey;
import java.util.ArrayList;

public class WindowBoxesVisionProcessor implements VisionProcessor {
    public Mat lastFrame;
    public Mat boxmaxes;
    public int Rows;
    public int Cols;
    public int colorindex=0;
    public int Height;
    public int Width;
    public Rect maxroi;

    public Rect startrect;

    public Paint rectPaint;

    public String color = "";

    @Override
    public void init(int width, int height, org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration calibration) {

        startrect=new Rect(0,0,width/2,height/2);
        maxroi=new Rect(0,0,width/2,height/2);


    }



    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

      lastFrame = frame.clone();

        return lastFrame;
    }

    @Override
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


    // NOTE: This is very hacked to only check the second row of 3.
    public Object[] topbox(int width,int height, int rows, int cols, AllianceColor color) {
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

    }

