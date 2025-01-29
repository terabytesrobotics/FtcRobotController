package org.firstinspires.ftc.teamcode.Processors;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.List;

public class SampleDetectVisionProcessor implements VisionProcessor {
    public Mat lastFrame;
    public ElapsedTime sampleTime = new ElapsedTime();
    public double lastSampleDelayTimeMillis = 0;
    public double lastSampleProcessingTimeMillis = 0;
    private static final int MAX_BLOCKS = 3;
    private static final Scalar RED_LOW1 =  new Scalar(0,120,70);
    private static final Scalar RED_HIGH1 = new Scalar(10,255,255);
    private static final Scalar RED_LOW2 =  new Scalar(170,120,70);
    private static final Scalar RED_HIGH2 = new Scalar(180,255,255);
    private static final Scalar BLUE_LOW  = new Scalar(90,150,70);
    private static final Scalar BLUE_HIGH = new Scalar(130,255,255);
    private static final Scalar YEL_LOW   = new Scalar(20,150,70);
    private static final Scalar YEL_HIGH  = new Scalar(30,255,255);

    @Override public void init(int width, int height, CameraCalibration calibration) {}

    @Override public Object processFrame(Mat frame, long captureTimeNanos) {
        lastSampleDelayTimeMillis = sampleTime.milliseconds();
        sampleTime.reset();
        Mat hsv = new Mat(); Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_BGR2HSV);

        Mat red1=new Mat(), red2=new Mat(), maskRed=new Mat(), maskBlue=new Mat(), maskYel=new Mat();
        Core.inRange(hsv, RED_LOW1, RED_HIGH1, red1);
        Core.inRange(hsv, RED_LOW2, RED_HIGH2, red2);
        Core.bitwise_or(red1, red2, maskRed);
        Core.inRange(hsv, BLUE_LOW, BLUE_HIGH, maskBlue);
        Core.inRange(hsv, YEL_LOW,  YEL_HIGH,  maskYel);

        Mat annotated = frame.clone();
        detectAndDrawLargestBlobs(maskRed,   annotated, new Scalar(0,0,255));
        detectAndDrawLargestBlobs(maskBlue,  annotated, new Scalar(255,0,0));
        detectAndDrawLargestBlobs(maskYel,   annotated, new Scalar(0,255,255));

        lastFrame = annotated;
        lastSampleProcessingTimeMillis = sampleTime.milliseconds();
        sampleTime.reset();
        return annotated;
    }

    private void detectAndDrawLargestBlobs(Mat mask, Mat drawOn, Scalar color) {
        Mat labels = new Mat(), stats = new Mat(), centroids = new Mat();
        int n = Imgproc.connectedComponentsWithStats(mask, labels, stats, centroids);
        List<Integer> idxs = new ArrayList<>();
        for(int i=1;i<n;i++){
            double area = stats.get(i,Imgproc.CC_STAT_AREA)[0];
            if(area>100) idxs.add(i);
        }
        idxs.sort((a,b)->Double.compare(
                stats.get(b,Imgproc.CC_STAT_AREA)[0],
                stats.get(a,Imgproc.CC_STAT_AREA)[0])
        );
        for(int i=0;i<idxs.size() && i<MAX_BLOCKS;i++){
            int lbl = idxs.get(i);
            Mat cmp = new Mat(); Core.compare(labels,new Scalar(lbl),cmp,Core.CMP_EQ);
            Moments m = Imgproc.moments(cmp,false);
            double cx = m.m10/m.m00, cy = m.m01/m.m00;
            double mu20 = m.mu20/m.m00, mu02 = m.mu02/m.m00, mu11 = m.mu11/m.m00;
            double angle = 0.5*Math.atan2(2*mu11, mu20 - mu02)*180/Math.PI;
            double a = mu20+mu02, b = Math.sqrt(4*mu11*mu11 + (mu20-mu02)*(mu20-mu02));
            double major = 4*Math.sqrt((a+b)/2), minor = 4*Math.sqrt((a-b)/2);
            RotatedRect r = new RotatedRect(new Point(cx, cy), new Size(major,minor), angle);
            Imgproc.ellipse(drawOn,r,color,2);
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
