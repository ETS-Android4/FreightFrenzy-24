package org.firstinspires.ftc.teamcode.robot.components.vision;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

public class CircleDetector {

    private static final String TAG = "CircleDetector";
    // Cache
    Mat circles = new Mat();


    int minAllowedX;
    int maxAllowedX;
    int minAllowedY;
    int maxAllowedY;

    public CircleDetector(int minAllowedX, int maxAllowedX, int minAllowedY, int maxAllowedY) {
        this.minAllowedX = minAllowedX;
        this.maxAllowedX = maxAllowedX;
        this.minAllowedY = minAllowedY;
        this.maxAllowedY = maxAllowedY;
    }

    public Mat process(Mat rgbaImage) {
        Imgproc.HoughCircles(rgbaImage, circles, Imgproc.CV_HOUGH_GRADIENT, 20, 130, 30, 100, 0, 0);
        return circles;
    }
}
