package org.firstinspires.ftc.teamcode.Processors;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;

public class SampleDetectVisionProcessor implements VisionProcessor {
    private static int MAX_BLOCKS = 1;
    public Mat lastFrame;
    public ElapsedTime sampleTime = new ElapsedTime();
    public double lastSampleDelayTimeMillis = 0;
    public double lastSampleProcessingTimeMillis = 0;

    public enum DetectableColor { RED, BLUE, YELLOW }
    private final EnumSet<DetectableColor> colorsToDetect;

    private static final Scalar RED_LOW1 =  new Scalar(0,120,70);
    private static final Scalar RED_HIGH1 = new Scalar(10,255,255);
    private static final Scalar RED_LOW2 =  new Scalar(170,120,70);
    private static final Scalar RED_HIGH2 = new Scalar(180,255,255);
    private static final Scalar BLUE_LOW  = new Scalar(90,150,70);
    private static final Scalar BLUE_HIGH = new Scalar(130,255,255);
    private static final Scalar YEL_LOW   = new Scalar(20,150,70);
    private static final Scalar YEL_HIGH  = new Scalar(30,255,255);

    public SampleDetectVisionProcessor(EnumSet<DetectableColor> colorsToDetect) {
        this.colorsToDetect = colorsToDetect;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {}

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        lastSampleDelayTimeMillis = sampleTime.milliseconds();
        sampleTime.reset();

        Mat rotated = new Mat();
        Core.rotate(frame, rotated, Core.ROTATE_90_COUNTERCLOCKWISE);
        frame = rotated;

        Mat hsv = new Mat();
        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV);
        Mat annotated = frame.clone();

        if (colorsToDetect.contains(DetectableColor.RED)) {
            Mat red1 = new Mat(), red2 = new Mat(), maskRed = new Mat();
            Core.inRange(hsv, RED_LOW1, RED_HIGH1, red1);
            Core.inRange(hsv, RED_LOW2, RED_HIGH2, red2);
            Core.bitwise_or(red1, red2, maskRed);
            detectAndDrawBlocks(maskRed, annotated, new Scalar(255, 255, 255));
        }

        if (colorsToDetect.contains(DetectableColor.BLUE)) {
            Mat maskBlue = new Mat();
            Core.inRange(hsv, BLUE_LOW, BLUE_HIGH, maskBlue);
            detectAndDrawBlocks(maskBlue, annotated, new Scalar(255, 255, 255));
        }

        if (colorsToDetect.contains(DetectableColor.YELLOW)) {
            Mat maskYellow = new Mat();
            Core.inRange(hsv, YEL_LOW, YEL_HIGH, maskYellow);
            detectAndDrawBlocks(maskYellow, annotated, new Scalar(255, 255, 255));
        }

        lastFrame = annotated;
        lastSampleProcessingTimeMillis = sampleTime.milliseconds();
        sampleTime.reset();
        return annotated;
    }

    private void detectAndDrawBlocks(Mat mask, Mat drawOn, Scalar color) {
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        contours.sort((c1, c2) -> Double.compare(Imgproc.contourArea(c2), Imgproc.contourArea(c1)));

        int count = 0;
        for (MatOfPoint contour : contours) {
            if (count >= MAX_BLOCKS) break;
            double area = Imgproc.contourArea(contour);
            if (area < 100) continue;
            RotatedRect ellipse = Imgproc.fitEllipse(new MatOfPoint2f(contour.toArray()));
            Imgproc.ellipse(drawOn, ellipse, color, 2);
            count++;
        }
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // Drawing is handled in processFrame now, nothing needed here.
        // Convert the lastFrame Mat to a Bitmap
        Bitmap bmp = Bitmap.createBitmap(lastFrame.cols(), lastFrame.rows(), Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(lastFrame, bmp);

        //Scale bitmap
        Bitmap scaledBmp = Bitmap.createScaledBitmap(bmp,onscreenWidth,onscreenHeight,false);

        // Draw the Bitmap on the Canvas
        canvas.drawBitmap(scaledBmp, 0, 0, null);
    }
}
