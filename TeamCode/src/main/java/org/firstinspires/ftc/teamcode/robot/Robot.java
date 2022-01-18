package org.firstinspires.ftc.teamcode.robot;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.components.CarouselSpinner;
import org.firstinspires.ftc.teamcode.robot.components.Intake;
import org.firstinspires.ftc.teamcode.robot.components.LED;
import org.firstinspires.ftc.teamcode.robot.components.OutPutter;
import org.firstinspires.ftc.teamcode.robot.components.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.robot.components.vision.OpenCVWebcam;
import org.firstinspires.ftc.teamcode.robot.components.vision.VslamCamera;
import org.firstinspires.ftc.teamcode.robot.operations.IntakeOperation;
import org.firstinspires.ftc.teamcode.robot.operations.Operation;
import org.firstinspires.ftc.teamcode.robot.operations.OperationThread;
import org.firstinspires.ftc.teamcode.robot.operations.OutputOperation;

/**
 * This class represents our robot.
 *
 * It supports the following controls:
 *  GamePad1:
 *         Left stick - drive, right stick - rotate
 *         x - abort pending operations
 *
 *         a - lowest arm level
 *         b - middle arm level
 *         y - top arm level
 *         Dpad Up - raise intake platform
 *         Dpad Down - lower intake platform
 *         Dpad Left - start intake
 *         Dpad right -
 *              if left trigger is pressed: expel
 *              otherwise - consume freight
 *
 *  GamePad2:
 *         Left stick - y axis - carousel speed
 *
 *         Dpad Up - raise output arm
 *         Dpad Down - lower output arm
 *
 *         Dpad Left -
 *          If right bumper is pressed
 *              Open Lid more
 *          Else
 *              retract output arm
 *         Dpad Right - extend output arm
 *          If right bumper is pressed
 *              Close Lid more
 *          Else
 *              extend output arm
 *
 *         Left trigger -
 *          If right bumper is pressed: open to capping position
 *          else open bucket
 *         Right trigger - close bucket

 *
 *         x - if left bumper is pressed, tell outputter that this is the correct intake level for intake
 *             otherwise
 *                  go to intake position
 *         a - if left bumper is pressed, tell outputter that this is the correct intake level for low level
 *             otherwise
 *                  go to lowest arm level
 *         b - if left bumper is pressed, tell outputter that this is the correct intake level for middle level
 *             otherwise
 *                  go to middle arm level
 *         y - if left bumper is pressed, tell outputter that this is the correct intake level for top level
 *             otherwise
 *                  go to top arm level
 *
 *
 */

public class Robot {

    Telemetry telemetry;
    private HardwareMap hardwareMap;
    Match match;

    OperationThread operationThreadPrimary;
    OperationThread operationThreadSecondary;
    OperationThread operationThreadTertiary;

    DriveTrain driveTrain;
    CarouselSpinner carouselSpinner;
    Intake intake;
    OutPutter outputter;
    LED led;

    OpenCVWebcam webcam;
    VslamCamera vslamCamera;

    boolean everythingButCamerasInitialized = false;

    //Our sensors etc.

    //our state
    String state = "pre-initialized";

    public Robot() {
        Log.d("SilverTitans", "Robot: got created");
    }

    /**
     * Initialize our robot
     * We set our alliance and our starting position based on finding a VuMark
     */
    public void init(HardwareMap hardwareMap, Telemetry telemetry, Match match) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.match = match;

        //initialize our components
        initCameras();
        initDriveTrain();
        this.carouselSpinner = new CarouselSpinner(hardwareMap);
        this.outputter = new OutPutter(hardwareMap);
        this.intake = new Intake(hardwareMap, outputter);
        this.led = new LED(hardwareMap);

        this.led.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);


        telemetry.addData("Status", "Creating operations thread, please wait");
        telemetry.update();

        Match.log("Started operations threads");
        this.operationThreadPrimary = new OperationThread(this, "Primary");
        operationThreadPrimary.start();
        this.operationThreadSecondary = new OperationThread(this, "Secondary");
        operationThreadSecondary.start();
        this.operationThreadTertiary = new OperationThread(this, "Tertiary");
        operationThreadTertiary.start();

        this.everythingButCamerasInitialized = true;
    }

    public void initDriveTrain() {
        //Create our drive train
        telemetry.addData("Status", "Initializing drive train, please wait");
        telemetry.update();
        this.driveTrain = new DriveTrain(hardwareMap, vslamCamera);
    }

    public void initCameras() {
        //initialize Vslam camera
        Match.log("Initializing VSLAM");
        telemetry.addData("Status", "Initializing VSLAM, please wait");
        telemetry.update();
        this.vslamCamera = new VslamCamera(hardwareMap);

        //initialize webcam
        Match.log("Initializing Webcam");
        telemetry.addData("Status", "Initializing Webcam, please wait");
        telemetry.update();
        this.webcam = new OpenCVWebcam();
        this.webcam.init(hardwareMap, telemetry, OpenCVWebcam.ELEMENT_COLOR_MIN, OpenCVWebcam.ELEMENT_COLOR_MAX);
    }


    /**
     * Stop the robot
     */
    public void stop() {
        //Stop all of our motors
        Match.log("Stopping robot");
        this.operationThreadPrimary.abort();
        this.operationThreadSecondary.abort();
        this.operationThreadTertiary.abort();
        this.driveTrain.stop();
        this.webcam.stop();
        this.outputter.stop();
        this.intake.stop();
        Match.log(("Robot stopped"));
    }

    public void queuePrimaryOperation(Operation operation) {
        this.operationThreadPrimary.queueUpOperation(operation);
    }
    public void queueSecondaryOperation(Operation operation) {
        this.operationThreadSecondary.queueUpOperation(operation);
    }
    public void queueTertiaryOperation(Operation operation) {
        this.operationThreadTertiary.queueUpOperation(operation);
    }

    /**
     * Returns the current x value of robot's center in mms
     * @return the current x position in mms
     */
    public double getCurrentX() {
        return this.vslamCamera.getPoseEstimate().getX()*Field.MM_PER_INCH;
    }

    /**
     * Returns the current y value of robot's center in mms
     * @return the current y position in mms
     */
    public double getCurrentY() {
        return this.vslamCamera.getPoseEstimate().getY()*Field.MM_PER_INCH;
    }

    /**
     * Returns the current heading of the robot in radians
     * @return the heading in radians
     */
    public double getCurrentTheta() {
        return AngleUnit.normalizeRadians(this.vslamCamera.getPoseEstimate().getHeading());}

    public boolean allOperationsCompleted() {
        return primaryOperationsCompleted() && secondaryOperationsCompleted() && tertiaryOperationsCompleted();
    }

    public boolean primaryOperationsCompleted() {
        return !this.operationThreadPrimary.hasEntries();
    }

    public boolean secondaryOperationsCompleted() {
        return !this.operationThreadSecondary.hasEntries();
    }

    public boolean tertiaryOperationsCompleted() {
        return !this.operationThreadTertiary.hasEntries();
    }

    public String getPosition() {
        return this.vslamCamera.getPoseEstimate().toString();
    }

    public boolean havePosition() {
        return vslamCamera.havePosition();
    }

    public String getState() {
        return this.state;
    }

    public void setState(String state) {
        this.state = state;
    }

    public boolean fullyInitialized() {
        return this.everythingButCamerasInitialized && this.vslamCamera.isInitialized();
    }


    public void handleGameControllers(Gamepad gamePad1, Gamepad gamePad2) {
        if (gamePad1.x) {
            this.operationThreadPrimary.abort();
            this.operationThreadSecondary.abort();
            this.operationThreadTertiary.abort();
        }
        this.handleDriveTrain(gamePad1);
        this.carouselSpinner.setSpeed(-gamePad2.left_stick_y);
        handleOutput(gamePad1, gamePad2);
        handleInput(gamePad1, gamePad2);
    }

    public void handleDriveTrain(Gamepad gamePad1) {
        if (this.primaryOperationsCompleted()) {
            double multiplier = gamePad1.right_trigger > 0.1 ? 1.0 : .6;
            double x = Math.pow(gamePad1.left_stick_x, 3) * multiplier; // Get left joystick's x-axis value.
            double y = -Math.pow(gamePad1.left_stick_y, 3) * multiplier; // Get left joystick's y-axis value.

            double rotation = Math.pow(gamePad1.right_stick_x, 3) * 0.5; // Get right joystick's x-axis value for rotation

            this.driveTrain.drive(Math.atan2(x, y), Math.hypot(x, y), rotation);
        }
    }

    public void handleInput(Gamepad gamePad1, Gamepad gamePad2) {
        //this.intake.setSpeed(gamePad2.left_stick_x);
        if (gamePad1.dpad_left) {
            intake.setForIntake();
        }
        else if (gamePad1.dpad_right) {
            if (gamePad1.left_trigger > .1) {
                queueTertiaryOperation(new IntakeOperation(intake, IntakeOperation.Type.Expel, "Expel"));
            }
            else {
                queueTertiaryOperation(new IntakeOperation(intake, IntakeOperation.Type.Consume, "Consume"));
            }
        }
        else if (gamePad1.dpad_up) {
            intake.raiseIntake();
        }
        else if (gamePad1.dpad_down) {
            intake.lowerIntake();
        }
    }

    public void handleOutput(Gamepad gamePad1, Gamepad gamePad2) {
        if (secondaryOperationsCompleted()) {
        /*
        game pads a, b, y control bucket levels for hub
        if gamepad2 left bumper is pressed, the levels are not set, the outputter is told that the new
        position for the specified level should be reset
         */
            if (gamePad1.a) {
                queueSecondaryOperation(new OutputOperation(outputter, intake, OutputOperation.Type.Level_Pickup, "Pickup level"));
            }
            if (gamePad2.a) {
                if (gamePad2.left_bumper) {
                    outputter.setEncoderOffset(OutputOperation.Type.Level_Low);
                }
                else {
                    queueSecondaryOperation(new OutputOperation(outputter, intake, OutputOperation.Type.Level_Low, "Low level"));
                }
            }
            if (gamePad1.b || gamePad2.b) {
                if (gamePad2.left_bumper) {
                    outputter.setEncoderOffset(OutputOperation.Type.Level_Middle);
                }
                else {
                    queueSecondaryOperation(new OutputOperation(outputter, intake, OutputOperation.Type.Level_Middle, "Middle level"));
                }
            }
            if (gamePad1.y) {
                queueSecondaryOperation(new OutputOperation(outputter, intake, OutputOperation.Type.Level_Llama, "Llama level"));
            }
            if (gamePad2.y) {
                if (gamePad2.left_bumper) {
                    outputter.setEncoderOffset(OutputOperation.Type.Level_High);
                }
                else {
                    queueSecondaryOperation(new OutputOperation(outputter, intake, OutputOperation.Type.Level_High, "High level"));
                }
            }
            //gamepad 2 x gets the bucket to intake mode
            if (gamePad2.x) {
                if (gamePad2.left_bumper) {
                    outputter.setEncoderOffset(OutputOperation.Type.Level_Intake);
                }
                else {
                    queueSecondaryOperation(new OutputOperation(outputter, intake, OutputOperation.Type.Level_Intake, "Intake level"));
                }
            }

            /*
            gamepad 2 dpad up/down raise/lower shoulder
             */
            if (gamePad2.dpad_up) {
                outputter.raiseAtShoulder();
            }
            else if (gamePad2.dpad_down) {
                outputter.lowerAtShoulder();
            }
            /*
            gamepad 2 dpad left/right retract/extend forearm at elbow or open/close lid more if right bumper is pressed
             */
            if (gamePad2.dpad_left) {
                if (gamePad2.right_bumper) {
                    outputter.openLidMore();
                }
                else {
                    outputter.retractForearm();
                }
            }
            else if (gamePad2.dpad_right) {
                if (gamePad2.right_bumper) {
                    outputter.closeLidMore();
                }
                else {
                    outputter.extendForearm();
                }
            }
        }

        //gamepad 2 left/right trigger open/close bucket
        //if right bumper is pressed, open goes to capping position
        if (gamePad2.left_trigger > .2) {
            if (gamePad2.right_bumper) {
                this.outputter.lidCappingPosition();
            }
            else {
                outputter.open();
            }
        }
        if (gamePad2.right_trigger > .2) {
            outputter.close();
        }
    }

    public void setInitialPose(Pose2d pose) {
        this.driveTrain.setLocalizer(vslamCamera);
        this.vslamCamera.setCurrentPose(pose);
    }

    public void reset() {
        if (this.driveTrain != null) {
            this.driveTrain.ensureWheelDirection();
            this.driveTrain.reset();
        }
        startVSLAM();
        //this.webcam.start();
    }

    public Pose2d getPose() {
        return this.vslamCamera.getPoseEstimate();
    }

    public void startVSLAM() {
        this.vslamCamera.start();
    }

    public OpenCVWebcam getWebcam() {
        return this.webcam;
    }

    public DriveTrain getDriveTrain() {
        return this.driveTrain;
    }

    public OutPutter getOutPutter() {
        return this.outputter;
    }

    public String getCarouselStatus() {
        return this.carouselSpinner.getStatus();
    }
    public String getOutputStatus() {
        return this.outputter.getStatus();
    }


    public void setPattern(RevBlinkinLedDriver.BlinkinPattern pattern) {
        this.led.setPattern(pattern);
    }
    public CarouselSpinner getCarouselSpinner() {
        return this.carouselSpinner;
    }

    public String getVSLAMStatus() {
        return this.vslamCamera.getStatus();
    }

    public T265Camera.PoseConfidence getPoseConfidence() {
        return vslamCamera.getPoseConfidence();
    }

    public int getBarCodeLevel() {
        return this.webcam.getBarCodeLevel();
    }

    public Intake getIntake() {
        return this.intake;
    }

    public String getIntakeStatus() {
        return this.intake.getStatus();
    }
}
