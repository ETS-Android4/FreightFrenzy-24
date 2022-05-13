/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.robot.components.vision;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.game.Alliance;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.RobotConfig;
import org.jetbrains.annotations.TestOnly;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

public class OpenCVWebcam {

    public static final double CAMERA_OFFSET_FRONT = 6.5;
    public static final int FOCAL_LENGTH = 1500;

    public static final Scalar BOX_COLOR_MIN = new Scalar(12, 50, 50);
    public static final Scalar BOX_COLOR_MAX = new Scalar(22, 255, 255);
    public static final Scalar SILVER_COLOR_MIN = new Scalar(220, 0, 128);
    public static final Scalar SILVER_COLOR_MAX = new Scalar(255, 20, 220);
    public static final Scalar ELEMENT_COLOR_MIN = new Scalar(0, 0, 205);
    public static final Scalar ELEMENT_COLOR_MAX = new Scalar(255, 45, 255);

    public static final int Y_PIXEL_COUNT = 1920;
    public static final int X_PIXEL_COUNT = 1080;
    public static final int MINIMUM_AREA = 200;

    OpenCvWebcam webcam;
    Pipeline pipeline;
    Scalar colorMin;
    Scalar colorMax;

    public static final Object synchronizer = new Object();
    @TestOnly
    public static void main(String[] args) {
        Imgcodecs imgcodecs = new Imgcodecs();
        Mat input = imgcodecs.imread("~/Desktop/teagan");
    }
    public void init(HardwareMap hardwareMap, Telemetry telemetry, Scalar colorMin, Scalar colorMax) {
        this.colorMin = colorMin;
        this.colorMax = colorMax;
        pipeline = new Pipeline();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, RobotConfig.WEBCAM_ID), cameraMonitorViewId);
        pipeline.setTelemetry(telemetry);
        webcam.setPipeline(pipeline);
        start();
    }

    public void start() {
        Match.log("Attempting to start " + RobotConfig.WEBCAM_ID + " usage");

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        //webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.showFpsMeterOnViewport(false);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                Match.log("Opened " + RobotConfig.WEBCAM_ID);

                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                webcam.startStreaming(Y_PIXEL_COUNT, X_PIXEL_COUNT, OpenCvCameraRotation.SIDEWAYS_LEFT);
                Match.log("Streaming started on " + RobotConfig.WEBCAM_ID);
            }

            @Override
            public void onError(int errorCode)
            {
                Match.log("Unable to open " + RobotConfig.WEBCAM_ID);
            }
        });
        //FtcDashboard.getInstance().startCameraStream(webcam, 10);
    }

    /**
     * Return the road runner pose of the object seen relative to the robot.
     * The robot is assumed to be facing the y axis with positive being in front of the robot.
     * Positive X axis is assumed to be running away to robot's right
     * @return Pose of object relative to the robot's center
     */
    public Pose2d getRelativeObjectPosition() {
        return null;
    }

    /**
     * Return the road runner pose of the object seen relative to the field.
     *
     * @return Pose of object in relation to the field
     */
    public Pose2d getAbsoluteObjectPosition(Pose2d robotPosition) {

        Pose2d relativeObjectPosition = getRelativeObjectPosition();
        if (relativeObjectPosition != null) {
            double heading = AngleUnit.normalizeRadians
                    (robotPosition.getHeading() + Math.toRadians(270) + relativeObjectPosition.getHeading());
            double hypotenuse = Math.hypot(relativeObjectPosition.getX(), relativeObjectPosition.getY());
            double x = Math.cos(heading) * hypotenuse;
            double y = Math.sin(heading) * hypotenuse;
            Vector2d objectAbsoluteVector = robotPosition.vec().plus(new Vector2d(x, y));
            return new Pose2d(objectAbsoluteVector, heading);
        }
        return null;
    }

    public void decrementMinX() {
        pipeline.contourDetector.decrementMinAllowedX();
    }

    public void incrementMinX() {
        pipeline.contourDetector.incrementMinAllowedX();
    }
    public void decrementMaxX() {
        pipeline.contourDetector.decrementMaxAllowedX();
    }
    public void incrementMaxX() {
        pipeline.contourDetector.incrementMaxAllowedX();
    }
    public void decrementMinY() {
        pipeline.contourDetector.decrementMinAllowedY();
    }
    public void incrementMinY() {
        pipeline.contourDetector.incrementMinAllowedY();
    }
    public void decrementMaxY() {
        pipeline.contourDetector.decrementMaxAllowedY();
    }
    public void incrementMaxY() {
        pipeline.contourDetector.incrementMaxAllowedY();
    }
    public void decrementMinHue() {pipeline.contourDetector.decrementMinHue();}
    public void decrementMaxHue() {pipeline.contourDetector.decrementMaxHue();}
    public void incrementMinHue() {pipeline.contourDetector.incrementMinHue();}
    public void incrementMaxHue() {pipeline.contourDetector.incrementMaxHue();}
    public void decrementMinSaturation() {pipeline.contourDetector.decrementMinSaturation();}
    public void decrementMaxSaturation() {pipeline.contourDetector.decrementMaxSaturation();}
    public void incrementMinSaturation() {pipeline.contourDetector.incrementMinSaturation();}
    public void incrementMaxSaturation() {pipeline.contourDetector.incrementMaxSaturation();}
    public void decrementMinValue() {pipeline.contourDetector.decrementMinValue();}
    public void decrementMaxValue() {pipeline.contourDetector.decrementMaxValue();}
    public void incrementMinValue() {pipeline.contourDetector.incrementMinValue();}
    public void incrementMaxValue() {pipeline.contourDetector.incrementMaxValue();}

    public String getBounds() {
        return (pipeline.contourDetector.getBounds());
    }

    public int getMinX() {
        return pipeline.contourDetector.contourMinX;
    }
    public int getMaxX() {
        return pipeline.contourDetector.contourMaxX;
    }
    public int getMinY() {
        return pipeline.contourDetector.contourMinY;
    }
    public int getMaxY() {
        return pipeline.contourDetector.contourMaxY;
    }

    public double getXPosition() {
        return pipeline.contourDetector.getXPosition();
    }
    public double getYPosition() {
        return pipeline.contourDetector.getYPosition();
    }

    /**
     * Return the distance to object in inches
     * @return
     */
    public double getDistanceToObjectFromCamera() { return pipeline.contourDetector.distanceToObjectFromCamera;}
    public double getDistanceToObjectFromCenter() { return pipeline.contourDetector.distanceFromCenterOfRobot();}
    public double getAngleToObjectFromCamera() { return pipeline.contourDetector.angleToObjectFromCamera;}
    public double getAngleToObjectFromCenter() { return pipeline.contourDetector.angleFromCenterOfRobot();}
    public Scalar getMean() {
        return pipeline.contourDetector.getMean();
    }

    public int getWidth() {
        return getMaxY() - getMinY();
    }

    public int getHeight() {
        return getMaxX() - getMinX();
    }

    public boolean seeingObject() {
        return pipeline.contourDetector.getLargestArea() > 0;
    }

    public double getLargestArea() {
        return pipeline.contourDetector.getLargestArea();
    }

    /**
     * Return where the team scoring element is on the bar code
     *
     * @return 1,2, or 3 depending on whether the object is on the left, center or right
     */
    public int getBarCodeLevel() {
        if (Match.getInstance().getAlliance() == Alliance.Color.RED) {
            if (seeingObject()) {
                if (getMinY() > 650) {
                    return 1;
                } else if (getMinY() > 200) {
                    return 2;
                }
            }
            return 3;
        } else {
            if (seeingObject()) {
                if (getMinY() > 1400) {
                    return 1;
                } else if (getMinY() > 1000) {
                    return 2;
                }
            }
            return 3;
        }
    }

    public class Pipeline extends OpenCvPipeline {
        Telemetry telemetry;
        public void setTelemetry(Telemetry telemetry) {
            this.telemetry = telemetry;
        }

        /*
         * Red, Blue and Green color constants
         */
        final Scalar RED = new Scalar(255, 0, 0);
        final Scalar GREEN = new Scalar(0, 255, 0);
        final Scalar BLUE = new Scalar(0, 0, 255);
        final Scalar SILVER = new Scalar(192, 192, 192);

        ContourDetector contourDetector = new ContourDetector(colorMin, colorMax,
                530, 1080, 0, 1920);

        @Override
        public Mat processFrame(Mat input) {
            synchronized (synchronizer) {
                List<MatOfPoint> contours = contourDetector.process(input);
                Imgproc.drawContours(input, contours, -1, GREEN, 5);
                MatOfPoint largestContour = contourDetector.getLargestContour();
                if (largestContour != null) {
                    List<MatOfPoint> largestContours = new ArrayList<>();
                    largestContours.add(largestContour);
                    Imgproc.drawContours(input, largestContours, -1, RED, 10);
                }
                Rect limitsRectangle = contourDetector.getAreaOfInterest();
                Imgproc.rectangle(input, limitsRectangle, RED, 6);
                Thread.yield();
                return input;
            }
        }
    }
     public void stop() {
        //webcam.stopStreaming();
     }
}