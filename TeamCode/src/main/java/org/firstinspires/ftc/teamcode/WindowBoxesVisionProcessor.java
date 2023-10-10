package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;

import java.lang.reflect.Array;
import java.security.PublicKey;
import java.util.ArrayList;

public class WindowBoxesVisionProcessor implements VisionProcessor {
    private Mat lastFrame;
    private Mat boxmaxes;
    private int Rows;
    private int Cols;
    private int colorindex;
    private int Height;
    private int Width;
    private Rect maxroi;



    @Override
    public void init(int width, int height, org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration calibration) {


    }



    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

      lastFrame = frame.clone();
        return frame;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

     /*   Paint rectPaint = new Paint();
        if (colorindex == 2){
            rectPaint.setColor(Color.RED);}
        else if (colorindex == 0){
            rectPaint.setColor(Color.BLUE);}
        rectPaint.setStyle(Paint.Style.STROKE);
        rectPaint.setStrokeWidth(scaleCanvasDensity * 4);

        canvas.drawRect(makeGraphicsRect(maxroi, scaleBmpPxToCanvasPx), rectPaint);
    */
    }



    public Object[] topbox (int width,int height, int rows, int cols, String color) {
        Mat evalFrame = lastFrame.clone();
        Mat boxes[][] = new Mat[rows][cols];
        double colorvals[][] = new double[rows][cols];
        boxmaxes = new Mat(rows, cols, CvType.CV_64FC1);

        Width = width;
        Height = height;
        Rows = rows;
        Cols = cols;
        int maxrow = -1;
        int maxcol = -1;
        double maxval = -1;


        if (color == "red") {
            colorindex = 2;


        } else if (color == "blue") {
            colorindex = 0;
        } else {
            System.err.println("Error: Color not recognized.");
        }
        // Create submats and store them in the boxes array
        int boxWidth = Width / Cols;
        int boxHeight = Height / Rows;

        for (int r = 0; r < Rows; r++) {
            //boxmaxes = null;
            for (int c = 0; c < Cols; c++) {

                Rect roi = new Rect(c * boxWidth, r * boxHeight, boxWidth, boxHeight);
                boxes[r][c] = new Mat(evalFrame, roi);

                // Calculate the average selected color intensity in the box
                Scalar avgColor = Core.mean(boxes[r][c]);

                colorvals[r][c] = avgColor.val[colorindex]; // Red channel (0=Blue, 1=Green, 2=Red)

                boxmaxes.put(r, c, colorvals[r][c]);

            }


        }
        Core.MinMaxLocResult result = Core.minMaxLoc(boxmaxes);

        maxval = result.maxVal;
        Point maxLoc = result.maxLoc;
        maxroi = new Rect(maxcol * boxWidth, maxrow * boxHeight, boxWidth, boxHeight);

        maxrow = (int) maxLoc.y;
        maxcol = (int) maxLoc.x;
        return new Object[]{maxrow, maxcol, maxval};







    }
    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }
}