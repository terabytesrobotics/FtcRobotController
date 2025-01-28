package org.firstinspires.ftc.teamcode.Processors;

import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class SampleTargetVisionProcessor implements VisionProcessor {
    private Mat lastFrame;
    private int frameCount = 0;
    private List<Mat> redMasks = new ArrayList<>();
    private List<Mat> blueMasks = new ArrayList<>();
    private List<Mat> yellowMasks = new ArrayList<>();
    public int numFramesToAverage;
    public List<String> targetColors;

    public void setParameters(int numFrames, List<String> colors) {
        this.numFramesToAverage = numFrames;
        this.targetColors = colors;
    }

    @Override
    public void init(int width, int height, org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration calibration) {
        // Initialization logic if needed

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        lastFrame = frame.clone();

        // Convert to HSV color space
        Mat hsvFrame = new Mat();
        Imgproc.cvtColor(lastFrame, hsvFrame, Imgproc.COLOR_RGB2HSV);

        // Masks for red, blue, and yellow
        Mat redMask = new Mat();
        Mat blueMask = new Mat();
        Mat yellowMask = new Mat();

        // Define HSV boundaries
        Scalar lowerRed = new Scalar(0, 70, 50);
        Scalar upperRed = new Scalar(10, 255, 255);
        Scalar lowerBlue = new Scalar(100, 150, 0);
        Scalar upperBlue = new Scalar(140, 255, 255);
        Scalar lowerYellow = new Scalar(20, 100, 100);
        Scalar upperYellow = new Scalar(30, 255, 255);

        // Detect colors
        if (targetColors.contains("Red")) {
            Core.inRange(hsvFrame, lowerRed, upperRed, redMask);
            redMasks.add(redMask);
        }
        if (targetColors.contains("Blue")) {
            Core.inRange(hsvFrame, lowerBlue, upperBlue, blueMask);
            blueMasks.add(blueMask);
        }
        if (targetColors.contains("Yellow")) {
            Core.inRange(hsvFrame, lowerYellow, upperYellow, yellowMask);
            yellowMasks.add(yellowMask);
        }

        frameCount++;

        // Wait until all frames are collected
        if (frameCount < numFramesToAverage) {
            return null;
        }

        // Average the masks
        Mat averagedRedMask = averageMasks(redMasks);
        Mat averagedBlueMask = averageMasks(blueMasks);
        Mat averagedYellowMask = averageMasks(yellowMasks);

        // Combine selected masks
        Mat combinedMask = new Mat();
        if (targetColors.contains("Red")) {
            Core.addWeighted(combinedMask, 1.0, averagedRedMask, 1.0, 0.0, combinedMask);
        }
        if (targetColors.contains("Blue")) {
            Core.addWeighted(combinedMask, 1.0, averagedBlueMask, 1.0, 0.0, combinedMask);
        }
        if (targetColors.contains("Yellow")) {
            Core.addWeighted(combinedMask, 1.0, averagedYellowMask, 1.0, 0.0, combinedMask);
        }

        // Reset frame count and masks
        frameCount = 0;
        redMasks.clear();
        blueMasks.clear();
        yellowMasks.clear();

        // Find contours
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(combinedMask, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        int frameArea = frame.rows() * frame.cols();
        double minArea = 0.05 * frameArea;
        Point frameCenter = new Point(frame.cols() / 2.0, frame.rows() / 2.0);

        // Determine the closest object to the center
        double closestDistance = Double.MAX_VALUE;
        String closestColor = "Unknown";
        Point closestPoint = new Point();
        double closestAngle = 0.0;

        for (MatOfPoint contour : contours) {
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            double peri = Imgproc.arcLength(contour2f, true);
            MatOfPoint2f approx = new MatOfPoint2f();
            Imgproc.approxPolyDP(contour2f, approx, 0.04 * peri, true);

            if (approx.total() == 4) {
                Rect boundingRect = Imgproc.boundingRect(new MatOfPoint(approx.toArray()));
                if (boundingRect.area() < minArea) continue;

                Point centerPoint = new Point(
                        boundingRect.x + boundingRect.width / 2.0,
                        boundingRect.y + boundingRect.height / 2.0
                );

                double distance = Math.hypot(centerPoint.x - frameCenter.x, centerPoint.y - frameCenter.y);
                if (distance < closestDistance) {
                    closestDistance = distance;

                    // Determine color
                    Mat maskROI = combinedMask.submat(boundingRect);
                    double redPixels = Core.sumElems(averagedRedMask.submat(boundingRect)).val[0] / 255.0;
                    double bluePixels = Core.sumElems(averagedBlueMask.submat(boundingRect)).val[0] / 255.0;
                    double yellowPixels = Core.sumElems(averagedYellowMask.submat(boundingRect)).val[0] / 255.0;

                    if (redPixels > bluePixels && redPixels > yellowPixels) {
                        closestColor = "Red";
                    } else if (bluePixels > redPixels && bluePixels > yellowPixels) {
                        closestColor = "Blue";
                    } else if (yellowPixels > redPixels && yellowPixels > bluePixels) {
                        closestColor = "Yellow";
                    }

                    closestPoint = centerPoint;
                    RotatedRect rotatedRect = Imgproc.minAreaRect(contour2f);
                    closestAngle = rotatedRect.angle;
                }
            }
        }

        // Return the result
        return new Object[]{
                closestColor,
                closestPoint.x,
                closestPoint.y,
                closestAngle
        };
    }

    private Mat averageMasks(List<Mat> masks) {
        if (masks.isEmpty()) {
            return new Mat();
        }
        Mat sum = Mat.zeros(masks.get(0).size(), masks.get(0).type());
        for (Mat mask : masks) {
            Core.add(sum, mask, sum);
        }
        Mat avg = new Mat();
        Core.divide(sum, new Scalar(masks.size()), avg);
        return avg;
    }

    @Override
    public void onDrawFrame(android.graphics.Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // Drawing logic if needed
    }
}

