package org.firstinspires.ftc.teamcode.robot.components.vision;

import org.firstinspires.ftc.teamcode.game.Field;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

public class ContourDetector {

    // Lower and Upper bounds for range checking in HSV color space
    private final Scalar mLowerBound = new Scalar(0);
    private final Scalar mUpperBound = new Scalar(0);
    private final Mat mSpectrum = new Mat();
    private MatOfPoint largestContour = null;
    List<MatOfPoint> contoursFound = new ArrayList<>();
    List<MatOfPoint> contoursInRange = new ArrayList<>();

    double contourMaxArea = 0;
    public int contourMinY = 9999, contourMinX = 9999;
    public int contourMaxY = 0, contourMaxX = 0;
    double distanceToObjectFromCamera, angleToObjectFromCamera;

    // Cache
    Mat mPyrDownMat = new Mat();
    Mat mHsvMat = new Mat();
    Mat mMask = new Mat();
    Mat mDilatedMask = new Mat();
    Mat mHierarchy = new Mat();

    public Scalar getMean() {
        return mean;
    }

    Scalar mean;
    Rect areaOfInterest;

    int minAllowedX;
    int maxAllowedX;
    int minAllowedY;
    int maxAllowedY;

    public ContourDetector(Scalar minColor, Scalar maxColor,
                           int minAllowedX, int maxAllowedX, int minAllowedY, int maxAllowedY) {
        this.setHsvColorRange(minColor, maxColor);
        this.minAllowedX = minAllowedX;
        this.maxAllowedX = maxAllowedX;
        this.minAllowedY = minAllowedY;
        this.maxAllowedY = maxAllowedY;
        setupAreaOfInterest();
    }

    private void setupAreaOfInterest() {
        areaOfInterest = new Rect(minAllowedX, minAllowedY, maxAllowedX-minAllowedX, maxAllowedY-minAllowedY);
    }

    public void setHsvColorRange(Scalar minHsvColor, Scalar maxHsvColor) {
        mLowerBound.val[0] = minHsvColor.val[0];
        mUpperBound.val[0] = maxHsvColor.val[0];

        mLowerBound.val[1] = minHsvColor.val[1];
        mUpperBound.val[1] = maxHsvColor.val[1];

        mLowerBound.val[2] = minHsvColor.val[2];
        mUpperBound.val[2] = maxHsvColor.val[2];

        mLowerBound.val[3] = 0;
        mUpperBound.val[3] = 255;

        Mat spectrumHsv = new Mat(1, (int)(maxHsvColor.val[0]-minHsvColor.val[0]), CvType.CV_8UC3);

        for (int j = 0; j < maxHsvColor.val[0]-minHsvColor.val[0]; j++) {
            byte[] tmp = {(byte)(maxHsvColor.val[0]+j), (byte)255, (byte)255};
            spectrumHsv.put(0, j, tmp);
        }

        Imgproc.cvtColor(spectrumHsv, mSpectrum, Imgproc.COLOR_HSV2RGB_FULL, 4);
    }

    public String getBounds() {
        return String.format(Locale.getDefault(),
                "Hue:%.0f-%.0f,Sat:%.0f-%.0f,Val:%.0f-%.0f, x:%d-%d, y:%d-%d",
                mLowerBound.val[0], mUpperBound.val[0], mLowerBound.val[1],
                mUpperBound.val[1], mLowerBound.val[2], mUpperBound.val[2],
                minAllowedX, maxAllowedX, minAllowedY, maxAllowedY);
    }

    public Rect getAreaOfInterest() {
        return areaOfInterest;
    }

    public List<MatOfPoint> process(Mat rgbaImage) {
        contoursFound.clear();
        contoursInRange.clear();
        largestContour = null;

        Imgproc.pyrDown(rgbaImage, mPyrDownMat);
        Imgproc.pyrDown(mPyrDownMat, mPyrDownMat);

        Imgproc.cvtColor(mPyrDownMat, mHsvMat, Imgproc.COLOR_RGB2HSV_FULL);

        Core.inRange(mHsvMat, mLowerBound, mUpperBound, mMask);
        Imgproc.dilate(mMask, mDilatedMask, new Mat());

        Imgproc.findContours(mDilatedMask, contoursFound, mHierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        contourMinX = contourMinY = 9999;
        contourMaxX = contourMaxY = 0;
        contourMaxArea = 0;
        for (MatOfPoint contour: contoursFound) {
            Rect boundingRectangle = Imgproc.boundingRect(contour);
            if (boundingRectangle.x*4 <= maxAllowedX && boundingRectangle.x*4 >= minAllowedX
                    && boundingRectangle.y*4 <= maxAllowedY && boundingRectangle.y*4 >=minAllowedY ) {
                double area = Imgproc.contourArea(contour);
                if (area >= OpenCVWebcam.MINIMUM_AREA) {
                    contoursInRange.add(contour);
                    if (area > contourMaxArea) {
                        contourMaxArea = area;
                        largestContour = contour;
                        Core.multiply(contour, new Scalar(4, 4), contour);
                        contourMinX = boundingRectangle.x * 4;
                        contourMaxX = boundingRectangle.x * 4 + boundingRectangle.width * 4;
                        contourMinY = boundingRectangle.y * 4;
                        contourMaxY = boundingRectangle.y * 4 + boundingRectangle.height * 4;
                        updateObjectPosition();
                    }
                    mean = Core.mean(contour);
                }
            }
        }
        return contoursInRange;
    }
    public MatOfPoint getLargestContour() {
        return largestContour;
    }
    void updateObjectPosition() {
        distanceToObjectFromCamera = getDistanceFromCamera();
        angleToObjectFromCamera = getAngleFromCamera();
    }

    public double getLargestArea() {
        return contourMaxArea;
    }

    public double getWidthInPixels() {
        if (getLargestArea() != 0) {
            return contourMaxY - contourMinY;
        }
        else {
            return 0;
        }
    }
    public double getHeightInPixels() {
        if (getLargestArea() != 0) {
            return contourMaxX - contourMinX;
        }
        else {
            return 0;
        }
    }

    public double getHeightWidthRatio() {
        if (contourMaxArea > 0) {
            return getHeightInPixels() / getWidthInPixels();
        }
        return 0;
    }
    public void decrementMinAllowedX() {
        this.minAllowedX = Math.max(minAllowedX - 1, 0);
        setupAreaOfInterest();
    }
    public void incrementMinAllowedX() {
        this.minAllowedX = Math.min(minAllowedX + 1, OpenCVWebcam.X_PIXEL_COUNT -1);
        setupAreaOfInterest();
    }
    public void decrementMaxAllowedX() {
        this.maxAllowedX = Math.max(maxAllowedX - 1, 0);
        setupAreaOfInterest();
    }
    public void incrementMaxAllowedX() {
        this.maxAllowedX = Math.min(maxAllowedX + 1, OpenCVWebcam.X_PIXEL_COUNT);
        setupAreaOfInterest();
    }

    public void decrementMinAllowedY() {
        this.minAllowedY = Math.max(minAllowedY - 1, 0);
        setupAreaOfInterest();
    }
    public void incrementMinAllowedY() {
        this.minAllowedY = Math.min(minAllowedY + 1, OpenCVWebcam.Y_PIXEL_COUNT-1);
        setupAreaOfInterest();
    }
    public void decrementMaxAllowedY() {
        this.maxAllowedY = Math.max(maxAllowedY - 1, 0);
        setupAreaOfInterest();
    }
    public void incrementMaxAllowedY() {
        this.maxAllowedY = Math.min(maxAllowedY + 1, OpenCVWebcam.Y_PIXEL_COUNT);
        setupAreaOfInterest();
    }

    public void decrementMinHue() {
        mLowerBound.val[0] = Math.max(mLowerBound.val[0] - 1, 0);
    }
    public void decrementMaxHue() {
        mUpperBound.val[0] = Math.max(mUpperBound.val[0] - 1, mLowerBound.val[0]);
    }
    public void incrementMinHue() {
        mLowerBound.val[0] = Math.min(mLowerBound.val[0] + 1, mUpperBound.val[0]);
    }
    public void incrementMaxHue() {
        mUpperBound.val[0] = Math.min(mUpperBound.val[0] + 1, 255);
    }

    public void decrementMinSaturation() {
        mLowerBound.val[1] = Math.max(mLowerBound.val[1] - 1, 0);
    }
    public void decrementMaxSaturation() {
        mUpperBound.val[1] = Math.max(mUpperBound.val[1] - 1, mLowerBound.val[1]);
    }
    public void incrementMinSaturation() {
        mLowerBound.val[1] = Math.min(mLowerBound.val[1] + 1, mUpperBound.val[1]);
    }
    public void incrementMaxSaturation() {
        mUpperBound.val[1] = Math.min(mUpperBound.val[1] + 1, 255);
    }
    public void decrementMinValue() {
        mLowerBound.val[2] = Math.max(mLowerBound.val[2] - 1, 0);
    }
    public void decrementMaxValue() {
        mUpperBound.val[2] = Math.max(mUpperBound.val[2] - 1, mLowerBound.val[2]);
    }
    public void incrementMinValue() {
        mLowerBound.val[2] = Math.min(mLowerBound.val[2] + 1, mUpperBound.val[2]);
    }
    public void incrementMaxValue() {
        mUpperBound.val[2] = Math.min(mUpperBound.val[2] + 1, 255);
    }
    /** Return the distance to the object from the camera in inches
     *
     * @return
     */
    private double getDistanceFromCamera() {
        int pixelWidth = (contourMaxY - contourMinY);
        return 1/Field.MM_PER_INCH * OpenCVWebcam.FOCAL_LENGTH / pixelWidth;
    }

    /** Returns the x position of the object in pixels
     *
     * @return
     */
    private int getXPixelPosition() {
        return (contourMaxY - contourMinY)/2 + contourMinY - OpenCVWebcam.Y_PIXEL_COUNT/2;
    }

    /** Returns the y position of the object seen in pixels
     * @return
     */
    public int getYPixelPosition() {
        return (contourMaxX - contourMinX)/2 + contourMinX;
    }

    /** Returns the x position of the object seen in inches
     * X and y coordinates are reversed from the point of view of the robot from the camera image
     *
     * Also camera's 0 is really at 1920/2
     *
     * @return
     */
    public double getXPosition() {
        return distanceToObjectFromCamera * Math.cos(angleToObjectFromCamera);
    }

    /** Returns the y position of the ring(s) seen in inches
     * X and y coordinates are reversed from the point of view of the robot from the camera image
     *
     * @return
     */
    public double getYPosition() {
        return distanceToObjectFromCamera * Math.sin(angleToObjectFromCamera);
    }

    /**
     * Return the angle of the object from the camera in radians
     * @return
     */
    public double getAngleFromCamera() {
        if (getHeightInPixels() != 0) {
            if (getXPixelPosition() != 0) {
                //x and y are reversed as we are in landscape mode
                ///y goes from left to right of robot, x from back to front
                return Math.atan2(getYPixelPosition()/OpenCVWebcam.X_PIXEL_COUNT,  getXPixelPosition()/OpenCVWebcam.Y_PIXEL_COUNT);
            }
            return Math.toRadians(90);
        }
        return 0;
    }
    /**
     * Returns distance in inches to the object from the center of the robot, given the camera offset from the center of the robot
     * @return
     */
    public double distanceFromCenterOfRobot() {
        if (angleToObjectFromCamera < Math.toRadians(90)) {
                return Math.sqrt(
                     Math.pow(OpenCVWebcam.CAMERA_OFFSET_FRONT, 2)
                            + Math.pow(distanceToObjectFromCamera, 2)
                            - 2*distanceToObjectFromCamera*OpenCVWebcam.CAMERA_OFFSET_FRONT*Math.cos(Math.toRadians(90) + angleToObjectFromCamera));
        }
        else {
            return Math.sqrt(
                    Math.pow(OpenCVWebcam.CAMERA_OFFSET_FRONT, 2)
                            + Math.pow(distanceToObjectFromCamera, 2)
                            - 2*distanceToObjectFromCamera*OpenCVWebcam.CAMERA_OFFSET_FRONT*Math.cos(Math.toRadians(270) - angleToObjectFromCamera));
        }
    }

    /**
     * Returns the angle in radians to the object from the center of the robot
     * @return
     */

    public double angleFromCenterOfRobot() {
        return Math.asin(distanceFromCenterOfRobot()*Math.sin(Math.toRadians(90) + angleToObjectFromCamera)/distanceToObjectFromCamera);
    }
}
