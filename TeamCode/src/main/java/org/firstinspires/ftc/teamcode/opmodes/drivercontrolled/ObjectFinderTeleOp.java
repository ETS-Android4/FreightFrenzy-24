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

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.components.vision.OpenCVWebcam;
import org.opencv.core.Scalar;

@TeleOp(name = "Phoebe: Freight Finder", group = "Phoebe")

public class ObjectFinderTeleOp extends DriverControlledOperation {
    public enum Mode {
        Hue, Saturation, Value
    }
    OpenCVWebcam webcam;
    boolean dpad2LeftPressed;
    boolean dpad2RightPressed;
    boolean dpad2UpPressed;
    boolean dpad2DownPressed;
    Mode mode;

    @Override
    public void init() {
        super.init();
        webcam = robot.getWebcam();
        robot.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
        mode = Mode.Hue;
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
        telemetry.addData("Mode", mode.toString() + " A/B/Y for H/S/V");
        telemetry.addData("Lt/Rt;Up/Dn", "+/- min; +/- max");
        telemetry.addData("LeftStick x/y", "Control min/max x");
        telemetry.addData("RightStick x/y", "Control min/max y");
        telemetry.addData("Bounds", webcam.getBounds());
        if (webcam.seeingObject()) {
            telemetry.addData("Object", "%d-%d, %d-%d",
                    webcam.getMinX(), webcam.getMaxX(), webcam.getMinY(), webcam.getMaxY()
            );
            telemetry.addData("Largest contour area", "%.0f", webcam.getLargestArea());
            telemetry.addData("Barcode", robot.getBarCodeLevel());
        }

        telemetry.update();

        //handle left joystick
        if (gamepad2.left_stick_x < -0.2) {
            webcam.decrementMinX();
        }
        if (gamepad2.left_stick_x > 0.2) {
            webcam.incrementMinX();
        }
        if (gamepad2.left_stick_y < -0.2) {
            webcam.decrementMinY();
        }
        if (gamepad2.left_stick_y > 0.2) {
            webcam.incrementMinY();
        }

        //handle right joystick
        if (gamepad2.right_stick_x < -0.2) {
            webcam.decrementMaxX();
        }
        if (gamepad2.right_stick_x > 0.2) {
            webcam.incrementMaxX();
        }
        if (gamepad2.right_stick_y < -0.2) {
            webcam.decrementMaxY();
        }
        if (gamepad2.right_stick_y > 0.2) {
            webcam.incrementMaxY();
        }

        //handle a, b and y
        if (gamepad2.a) {
            mode = Mode.Hue;
        }
        else if (gamepad2.b) {
            mode = Mode.Saturation;
        }
        else if (gamepad2.y) {
            mode = Mode.Value;
        }

        //handle dpad to control mode specific filters filters
        if (gamepad2.dpad_left) {
            if (!dpad2LeftPressed) {
                switch (mode) {
                    case Hue: {
                        webcam.decrementMinHue();
                        break;
                    }
                    case Saturation: {
                        webcam.decrementMinSaturation();
                        break;
                    }
                    case Value: {
                        webcam.decrementMinValue();
                        break;
                    }
                }
                dpad2LeftPressed = true;
            }
        }
        else {
            dpad2LeftPressed = false;
        }

        if (gamepad2.dpad_right) {
            if (!dpad2RightPressed) {
                switch (mode) {
                    case Hue: {
                        webcam.incrementMinHue();
                        break;
                    }
                    case Saturation: {
                        webcam.incrementMinSaturation();
                        break;
                    }
                    case Value: {
                        webcam.incrementMinValue();
                        break;
                    }
                }
                dpad2RightPressed = true;
            }
        }
        else {
            dpad2RightPressed = false;
        }

        if (gamepad2.dpad_down) {
            if (!dpad2DownPressed) {
                switch (mode) {
                    case Hue: {
                        webcam.decrementMaxHue();
                        break;
                    }
                    case Saturation: {
                        webcam.decrementMaxSaturation();
                        break;
                    }
                    case Value: {
                        webcam.decrementMaxValue();
                        break;
                    }
                }
                dpad2DownPressed = true;
            }
        }
        else {
            dpad2DownPressed = false;
        }
        if (gamepad2.dpad_up) {
            if (!dpad2UpPressed) {
                switch (mode) {
                    case Hue: {
                        webcam.incrementMaxHue();
                        break;
                    }
                    case Saturation: {
                        webcam.incrementMaxSaturation();
                        break;
                    }
                    case Value: {
                        webcam.incrementMaxValue();
                        break;
                    }
                }
                dpad2UpPressed = true;
            }
        }
        else {
            dpad2UpPressed = false;
        }
    }

}