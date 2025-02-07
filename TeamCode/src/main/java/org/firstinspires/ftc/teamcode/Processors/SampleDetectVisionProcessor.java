package org.firstinspires.ftc.teamcode.Processors;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.roadrunner.util.Angle;
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
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;

public class SampleDetectVisionProcessor implements VisionProcessor {
    private static int MAX_BLOCKS = 1;
    public Mat lastFrame;
    public ElapsedTime sampleTime = new ElapsedTime();
    public double lastSampleDelayTimeMillis = 0;
    public double lastSampleProcessingTimeMillis = 0;

    // Holds the best detected ellipse’s angle (in degrees)
    public volatile Double detectedEllipseAngle = null;
    // Used to choose the best candidate in the frame.
    private double bestEllipseArea = 0;

    public enum DetectableColor { RED, BLUE, YELLOW }
    private final EnumSet<DetectableColor> colorsToDetect;

    // HSV thresholds for our colors.
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
    public void init(int width, int height, CameraCalibration calibration) {
        // Initialization if needed.
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // Reset candidate information.
        bestEllipseArea = 0;
        detectedEllipseAngle = null;

        lastSampleDelayTimeMillis = sampleTime.milliseconds();
        sampleTime.reset();

        // Convert the frame from RGB to HSV.
        Mat hsv = new Mat();
        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV);

        // Draw ellipses directly on the original frame.
        if (colorsToDetect.contains(DetectableColor.RED)) {
            Mat red1 = new Mat(), red2 = new Mat(), maskRed = new Mat();
            Core.inRange(hsv, RED_LOW1, RED_HIGH1, red1);
            Core.inRange(hsv, RED_LOW2, RED_HIGH2, red2);
            Core.bitwise_or(red1, red2, maskRed);
            detectAndDrawBlocks(maskRed, frame, new Scalar(255, 0, 0));
        }

        if (colorsToDetect.contains(DetectableColor.BLUE)) {
            Mat maskBlue = new Mat();
            Core.inRange(hsv, BLUE_LOW, BLUE_HIGH, maskBlue);
            detectAndDrawBlocks(maskBlue, frame, new Scalar(0, 0, 255));
        }

        if (colorsToDetect.contains(DetectableColor.YELLOW)) {
            Mat maskYellow = new Mat();
            Core.inRange(hsv, YEL_LOW, YEL_HIGH, maskYellow);
            detectAndDrawBlocks(maskYellow, frame, new Scalar(255, 255, 0));
        }

        // Save the modified frame for onDrawFrame().
        lastFrame = frame;
        lastSampleProcessingTimeMillis = sampleTime.milliseconds();
        sampleTime.reset();
        return frame;
    }

    /**
     * Finds contours in the mask and draws fitted ellipses on the provided image.
     *
     * @param mask   The binary mask where the desired color is white.
     * @param drawOn The image (original frame) on which to draw the ellipses.
     * @param color  The color to use for drawing.
     */
    private void detectAndDrawBlocks(Mat mask, Mat drawOn, Scalar color) {
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        // Sort contours by area (largest first)
        contours.sort((c1, c2) -> Double.compare(Imgproc.contourArea(c2), Imgproc.contourArea(c1)));

        int count = 0;
        for (MatOfPoint contour : contours) {
            if (count >= MAX_BLOCKS) break;
            double area = Imgproc.contourArea(contour);
            // Skip small contours.
            if (area < 100) continue;
            // fitEllipse requires at least 5 points.
            if (contour.toArray().length < 5) continue;

            RotatedRect ellipse = Imgproc.fitEllipse(new MatOfPoint2f(contour.toArray()));
            // Draw the ellipse outline with a thicker stroke.
            Imgproc.ellipse(drawOn, ellipse, color, 3);
            // Optionally, draw the center point.
            Imgproc.circle(drawOn, ellipse.center, 4, color, -1);

            // Save the angle of the largest detected ellipse.
            if (area > bestEllipseArea) {
                bestEllipseArea = area;
                detectedEllipseAngle = -(ellipse.angle - 90);
            }
            count++;
        }
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // Convert the modified frame to a Bitmap and draw it.
        Bitmap bmp = Bitmap.createBitmap(lastFrame.cols(), lastFrame.rows(), Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(lastFrame, bmp);
        Bitmap scaledBmp = Bitmap.createScaledBitmap(bmp, onscreenWidth, onscreenHeight, false);
        canvas.drawBitmap(scaledBmp, 0, 0, null);
    }
}
