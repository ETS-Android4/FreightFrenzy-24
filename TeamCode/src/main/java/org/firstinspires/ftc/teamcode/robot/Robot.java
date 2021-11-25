package org.firstinspires.ftc.teamcode.robot;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.game.Alliance;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.components.CarouselSpinner;
import org.firstinspires.ftc.teamcode.robot.components.Intake;
import org.firstinspires.ftc.teamcode.robot.components.LED;
import org.firstinspires.ftc.teamcode.robot.components.OutPutter;
import org.firstinspires.ftc.teamcode.robot.components.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.robot.components.vision.OpenCVWebcam;
import org.firstinspires.ftc.teamcode.robot.components.vision.VslamCamera;
import org.firstinspires.ftc.teamcode.robot.operations.Operation;
import org.firstinspires.ftc.teamcode.robot.operations.OperationThread;

/**
 * This class represents our robot.
 *
 * It supports the following controls:
 *  GamePad1:
 *         Left stick - drive, right stick - rotate
 *         x - abort pending operations
 *         a - lowest arm level
 *         b - middle arm level
 *         y - top arm level
 *         Dpad Up - raise output arm
 *         Dpad Down - lower output arm
 *         Dpad Left -
 *              if right trigger is pressed: lower intake platform
 *              other wise - set to input position
 *         Dpad right -
 *              if right trigger is pressed: raise intake platform
 *              otherwise - set to output position
 *
 *  GamePad2:
 *         Left stick - y axis - carousel speed
 *         Left stick - x axis - intake speed
 *         Right stick - y axis - outPutter speed
 *         Dpad Up - raise capping arm
 *         Dpad Down - lower capping arm
 *         Left trigger - wind capping servo
 *         Right trigger - unwind capping servo
 *         Left bumper - retract output
 *         x - fold arm
 *         a - lowest arm level
 *         b - middle arm level
 *         y - top arm level
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
        this.intake = new Intake(hardwareMap);
        this.outputter = new OutPutter(hardwareMap);
        this.led = new LED(hardwareMap);

        if (match.getAlliance() == Alliance.Color.RED) {
            this.led.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }
        else {
            this.led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        }


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
        //initialize webcam
        Match.log("Initializing Webcam");
        telemetry.addData("Status", "Initializing Webcam, please wait");
        telemetry.update();
        this.webcam = new OpenCVWebcam();
        this.webcam.init(hardwareMap, telemetry, OpenCVWebcam.ELEMENT_COLOR_MIN, OpenCVWebcam.ELEMENT_COLOR_MAX);

        //initialize Vslam camera
        Match.log("Initializing VSLAM");
        telemetry.addData("Status", "Initializing VSLAM, please wait");
        telemetry.update();
        this.vslamCamera = new VslamCamera(hardwareMap);
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
        //this.intake.stop();
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
        //this.intake.setSpeed(-gamePad2.right_stick_y);
        handleOutput(gamePad1, gamePad2);
        handleInput(gamePad1, gamePad2);
    }

    public void handleDriveTrain(Gamepad gamePad1) {
        if (this.primaryOperationsCompleted()) {
            double multiplier = 0.6;
            double x = Math.pow(gamePad1.left_stick_x, 3) * multiplier; // Get left joystick's x-axis value.
            double y = -Math.pow(gamePad1.left_stick_y, 3) * multiplier; // Get left joystick's y-axis value.

            double rotation = Math.pow(gamePad1.right_stick_x, 3) * 0.5; // Get right joystick's x-axis value for rotation

            this.driveTrain.drive(Math.atan2(x, y), Math.hypot(x, y), rotation);
        }
    }

    public void handleInput(Gamepad gamePad1, Gamepad gamePad2) {
        this.intake.setSpeed(gamePad2.left_stick_x);
        if (gamePad1.dpad_left) {
            if (gamePad1.right_trigger > .1) {
                intake.lowerIntake();
                outputter.fold();
            }
            else {
                intake.setForIntake();
            }
        }
        else if (gamePad1.dpad_right) {
            if (gamePad1.right_trigger > .1) {
                intake.raiseIntake();
            }
            else {
                intake.setForOutput();
            }
        }
    }

    public void handleOutput(Gamepad gamePad1, Gamepad gamePad2) {
        if (gamePad2.left_bumper) {
            outputter.retract();
        }
        if (gamePad1.dpad_up) {
            outputter.raiseArm();
        }
        else if (gamePad1.dpad_down) {
            outputter.lowerArm();
        }
        if (gamePad1.a || gamePad2.a) {
            outputter.setLowPosition();
        }
        if (gamePad1.b || gamePad2.b) {
            outputter.setMiddlePosition();
        }
        if (gamePad1.y || gamePad2.y) {
            outputter.setHighPosition();
        }
        if (gamePad2.x) {
            outputter.fold();
        }

        this.outputter.setSpeed(-gamePad2.right_stick_y);
    }

    public boolean setInitialPose(Pose2d pose) {
        this.driveTrain.setLocalizer(vslamCamera);
        return this.vslamCamera.setCurrentPose(pose);
    }

    public void reset() {
        if (this.driveTrain != null) {
            this.driveTrain.ensureWheelDirection();
        }
        startVSLAM();
        this.webcam.start();
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

    public String getIntakeStatus() {
        return this.intake.getStatus();
    }

    public void setInitialOutputPosition() {
        this.outputter.setInitialPosition();
    }
}
