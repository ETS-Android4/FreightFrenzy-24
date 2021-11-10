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

package org.firstinspires.ftc.teamcode.opmodes.drivercontrolled;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.components.vision.OpenCVWebcam;
import org.opencv.core.Scalar;

import java.util.Locale;

@TeleOp(name = "Phoebe: Freight Finder", group = "Phoebe")

public class ObjectFinderTeleOp extends DriverControlledOperation {
    OpenCVWebcam webcam;
    boolean dpad1LeftPressed, dpad2LeftPressed, dpad1RightPressed, dpad2RightPressed,
        dpad1UpPressed, dpad2UpPressed, dpad1DownPressed, dpad2DownPressed;
    @Override
    public void init() {
        super.init();
        webcam = robot.getWebcam();
    }
    public void startStreaming(Scalar colorMin, Scalar colorMax) {
        webcam.init(hardwareMap, telemetry, colorMin, colorMax);
    }
    @Override
    public void start() {
        startStreaming(OpenCVWebcam.BOX_COLOR_MIN, OpenCVWebcam.BOX_COLOR_MAX);
    }
    @Override
    public void loop() {
        if (webcam.seeingObject()) {
            telemetry.addData("Object", String.format(Locale.getDefault(), "%d-%d, %d-%d",
                    webcam.getMinX(), webcam.getMaxX(), webcam.getMinY(), webcam.getMaxY())
            );
        }
        telemetry.addData("Bounds", webcam.getBounds());

        telemetry.update();
        if (gamepad2.left_stick_x < -0.2) {
            webcam.decrementMinX();
        }
        if (gamepad2.left_stick_x > 0.2) {
            webcam.incrementMinX();
        }
        if (gamepad2.right_stick_x < -0.2) {
            webcam.decrementMaxX();
        }
        if (gamepad2.right_stick_x > 0.2) {
            webcam.incrementMaxX();
        }

        if (gamepad2.left_stick_y < -0.2) {
            webcam.decrementMinY();
        }
        if (gamepad2.left_stick_y > 0.2) {
            webcam.incrementMinY();
        }
        if (gamepad2.right_stick_y < -0.2) {
            webcam.decrementMaxY();
        }
        if (gamepad2.right_stick_y > 0.2) {
            webcam.incrementMaxY();
        }

        //handle left gamepad dpad to control saturation filters
        if (gamepad1.dpad_left) {
            if (!dpad1LeftPressed) {
                webcam.decrementMinSaturation();
                dpad1LeftPressed = true;
            }
        }
        else {
            dpad1LeftPressed = false;
        }

        if (gamepad1.dpad_right) {
            if (!dpad1RightPressed) {
                webcam.incrementMinSaturation();
                dpad1RightPressed = true;
            }
        }
        else {
            dpad1RightPressed = false;
        }

        if (gamepad1.dpad_down) {
            if (!dpad1DownPressed) {
                webcam.decrementMaxSaturation();
                dpad1DownPressed = true;
            }
        }
        else {
            dpad1DownPressed = false;
        }
        if (gamepad1.dpad_up) {
            if (!dpad1UpPressed) {
                webcam.incrementMaxSaturation();
                dpad1UpPressed = true;
            }
        }
        else {
            dpad1UpPressed = false;
        }

        //handle dpad of second game controller
        if (gamepad2.dpad_left) {
            if (!dpad2LeftPressed) {
                webcam.decrementMinHue();
                dpad2LeftPressed = true;
            }
        }
        else {
            dpad2LeftPressed = false;
        }

        if (gamepad2.dpad_right) {
            if (!dpad2RightPressed) {
                webcam.incrementMinHue();
                dpad2RightPressed = true;
            }
        }
        else {
            dpad2RightPressed = false;
        }

        if (gamepad2.dpad_down) {
            if (!dpad2DownPressed) {
                webcam.decrementMaxHue();
                dpad2DownPressed = true;
            }
        }
        else {
            dpad2DownPressed = false;
        }
        if (gamepad2.dpad_up) {
            if (!dpad2UpPressed) {
                webcam.incrementMaxHue();
                dpad2UpPressed = true;
            }
        }
        else {
            dpad2UpPressed = false;
        }
    }

}