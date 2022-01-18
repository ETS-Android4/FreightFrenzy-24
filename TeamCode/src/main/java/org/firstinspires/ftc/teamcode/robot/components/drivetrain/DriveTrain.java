package org.firstinspires.ftc.teamcode.robot.components.drivetrain;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.components.vision.VslamCamera;
import org.firstinspires.ftc.teamcode.robot.operations.BearingOperation;
import org.firstinspires.ftc.teamcode.robot.operations.DriveForDistanceOperation;
import org.firstinspires.ftc.teamcode.robot.operations.DriveToPositionOperation;
import org.firstinspires.ftc.teamcode.robot.operations.FollowTrajectory;
import org.firstinspires.ftc.teamcode.robot.operations.StrafeLeftForDistanceOperation;
import org.firstinspires.ftc.teamcode.robot.operations.StrafeLeftForDistanceWithHeadingOperation;
import org.firstinspires.ftc.teamcode.robot.operations.TurnAntiClockwiseOperation;
import org.firstinspires.ftc.teamcode.robot.operations.TurnClockwiseOperation;

import java.util.Date;
import java.util.Locale;

/**
 * Created by Silver Titans on 10/26/17.
 */

public class DriveTrain extends SampleMecanumDrive {
    public static final int WITHIN_RANGE = 30;
    public static final double     P_TURN_COEFFICIENT            = 1;     // Larger is more responsive, but also less stable
    public static final double P_DRIVE_COEFFICIENT = 0.025;     // Larger is more responsive, but also less stable

    int lastLeftFrontEncoderValue;
    int lastLeftRearEncoderValue;
    int lastRightFrontEncoderValue;
    int lastRightRearEncoderValue;

    public int getLastLeftFrontEncoderValue() {
        return lastLeftFrontEncoderValue;
    }

    public int getLastLeftRearEncoderValue() {
        return lastLeftRearEncoderValue;
    }

    public int getLastRightFrontEncoderValue() {
        return lastRightFrontEncoderValue;
    }

    public int getLastRightRearEncoderValue() {
        return lastRightRearEncoderValue;
    }


    public DriveTrain(HardwareMap hardwareMap, VslamCamera camera) {
        super(hardwareMap, camera);
    }

    /** Set power of left front motor
     *
     * @param power - the power to set
     *
     */
    public void setLeftFrontPower(double power) {
        //super.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        super.leftFront.setPower(power);
    }

    /**
     * Set power of right front motor
     * @param power - the power to set
     */
    public void setRightFrontPower(double power) {
        //super.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        super.rightFront.setPower(power);
    }

    /** Set power of left rear motor
     *
     * @param power - the power to set
     *
     */
    public void setLeftRearPower(double power) {
        //super.leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        super.leftRear.setPower(power);
    }

    /**
     * Set power of right rear motor
     * @param power - the power to set
     */
    public void setRightRearPower(double power) {
        //super.rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        super.rightRear.setPower(power);
    }

    public void handleOperation(FollowTrajectory trajectoryOperation) {
        Pose2d currentPose = getPoseEstimate();
        Match.log("Starting " + trajectoryOperation.getTitle() + ": "  + trajectoryOperation.getTrajectory().start() + "->" + trajectoryOperation.getTrajectory().end()
                + " at " + currentPose);
        super.followTrajectoryAsync(trajectoryOperation.getTrajectory());
    }

    public void handleOperation (DriveToPositionOperation operation) {
        try {
            Pose2d currentPose = getPoseEstimate();
            Date start = new Date();
            Thread createTrajectoryThread = new Thread(() -> {
                Trajectory trajectory = accurateTrajectoryBuilder(currentPose)
                        .splineToLinearHeading(operation.getDesiredPose(), operation.getDesiredPose().getHeading())
                        .build();
                Match.log("Starting " + operation.getTitle() + ": " + trajectory.start() + "->" + trajectory.end()
                        + " at " + currentPose + ", build took " + (new Date().getTime() - start.getTime()) + " mSecs");
                //set trajectory in operation
                operation.setTrajectory(trajectory);
                //start following trajectory
                super.followTrajectoryAsync(trajectory);
                //mark that trajectory has been started
                operation.setTrajectoryStarted(true);
            });
            createTrajectoryThread.start();
        }
        catch (Throwable e) {
            Match.log("Error starting drive to position");
            RobotLog.logStackTrace(e);
            operation.setAborted(true);
        }
    }

    public void handleOperation (BearingOperation operation) {
        try {
            Pose2d currentPose = getPoseEstimate();
            Pose2d desiredPose =
                    currentPose.plus(
                            new Pose2d(.5,
                                    .5,
                                    Math.toRadians(operation.getDesiredBearing())-currentPose.getHeading()));
            Date start = new Date();
            Thread createTrajectoryThread = new Thread(() -> {
                Trajectory trajectory = trajectoryBuilder(currentPose)
                        .splineToLinearHeading(desiredPose, 0)
                        .build();
                Match.log("Starting " + operation.getTitle() + ": " + trajectory.start() + "->" + trajectory.end()
                        + " at " + currentPose + ", build took " + (new Date().getTime() - start.getTime()) + " mSecs");
                //set trajectory in operation
                operation.setTrajectory(trajectory);
                //start following trajectory
                super.followTrajectoryAsync(trajectory);
                //mark that trajectory has been started
                operation.setTrajectoryStarted(true);
            });
            createTrajectoryThread.start();
        }
        catch (Throwable e) {
            Match.log("Error starting drive to position");
            RobotLog.logStackTrace(e);
            operation.setAborted(true);
        }
    }
    /**
     * Handle operation to drive for the specified distance in the direction the robot is facing
     * @param operation
     *
     * We do this by computing how much each wheel must be rotated to travel the specified distance
     * and then commanding the motors to reach the new desired encoder values
     *
     */
    public void handleOperation(DriveForDistanceOperation operation) {
        stop();

        int encoderChange = DriveConstants.mmToEncoderTicks(operation.getDistance());
        this.leftFront.setTargetPosition(leftFront.getCurrentPosition() + encoderChange);
        this.rightFront.setTargetPosition(rightFront.getCurrentPosition() + encoderChange);
        this.leftRear.setTargetPosition(leftRear.getCurrentPosition() + encoderChange);
        this.rightRear.setTargetPosition(rightRear.getCurrentPosition() + encoderChange);

        this.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.leftFront.setPower(operation.getSpeed());
        this.rightFront.setPower(operation.getSpeed());
        this.leftRear.setPower(operation.getSpeed());
        this.rightRear.setPower(operation.getSpeed());
    }

    /**
     * Handle operation to turn clockwise for the specified distance
     * @param operation
     *
     * We do this by computing how much each wheel must be rotated to travel the specified distance
     * and then commanding the motors to reach the new desired encoder values
     *
     */
    public void handleOperation(TurnClockwiseOperation operation) {
        stop();

        int encoderChange = DriveConstants.mmToEncoderTicks(operation.getDistance());
        this.rightFront.setTargetPosition(leftFront.getCurrentPosition() + encoderChange/2);
        this.rightRear.setTargetPosition(rightRear.getCurrentPosition() + encoderChange/2);
        this.leftFront.setTargetPosition(leftFront.getCurrentPosition() + encoderChange);
        this.leftRear.setTargetPosition(rightRear.getCurrentPosition() + encoderChange);

        this.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.rightFront.setPower(operation.getSpeed()/2);
        this.rightRear.setPower(operation.getSpeed()/2);
        this.leftFront.setPower(operation.getSpeed());
        this.leftRear.setPower(operation.getSpeed());
    }


    /**
     * Handle operation to turn anti-clockwise for the specified distance
     * @param operation
     *
     * We do this by computing how much each wheel must be rotated to travel the specified distance
     * and then commanding the motors to reach the new desired encoder values
     *
     */
    public void handleOperation(TurnAntiClockwiseOperation operation) {
        stop();

        int encoderChange = DriveConstants.mmToEncoderTicks(operation.getDistance());
        this.rightFront.setTargetPosition(leftFront.getCurrentPosition() + encoderChange);
        this.rightRear.setTargetPosition(rightRear.getCurrentPosition() + encoderChange);
        this.leftFront.setTargetPosition(leftFront.getCurrentPosition() + encoderChange/2);
        this.leftRear.setTargetPosition(rightRear.getCurrentPosition() + encoderChange/2);

        this.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.rightFront.setPower(operation.getSpeed());
        this.rightRear.setPower(operation.getSpeed());
        this.leftFront.setPower(operation.getSpeed()/2);
        this.leftRear.setPower(operation.getSpeed()/2);
    }

    /**
     * Handle operation to strafe left for the specified distance perpendicular to the direction the robot is facing
     * @param operation
     *
     * We do this by computing how much each wheel must be rotated to travel the specified distance
     * and then commanding the motors to reach the new desired encoder values
     *
     * We make the left front and right rear motors move forward while making the right front and left rear
     * motors propel backwards
     *
     */
    public void handleOperation(StrafeLeftForDistanceOperation operation) {
        stop();
        int encoderChange = DriveConstants.mmToEncoderTicks(operation.getDistance() * 1.05);
        this.leftFront.setTargetPosition(leftFront.getCurrentPosition() - encoderChange);
        this.rightFront.setTargetPosition(rightFront.getCurrentPosition() + encoderChange);
        this.leftRear.setTargetPosition(leftRear.getCurrentPosition() + encoderChange);
        this.rightRear.setTargetPosition(rightRear.getCurrentPosition() - encoderChange);

        this.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.leftFront.setPower(operation.getSpeed());
        this.rightFront.setPower(operation.getSpeed());
        this.leftRear.setPower(operation.getSpeed());
        this.rightRear.setPower(operation.getSpeed());
    }

    /**
     * Handle operation to strafe for the specified distance perpendicular to the direction the robot is facing
     * @param operation
     *
     * We do this by computing how much each wheel must be rotated to travel the specified distance
     * and then commanding the motors to reach the new desired encoder values
     *
     * We make the left front and right rear motors move forward while making the right front and left rear
     * motors propel backwards
     *
     */
    public void handleOperation(StrafeLeftForDistanceWithHeadingOperation operation) {
        stop();
        int encoderChange = DriveConstants.mmToEncoderTicks(operation.getDistance() * 1.05);
        this.leftFront.setTargetPosition(leftFront.getCurrentPosition() - encoderChange);
        this.rightFront.setTargetPosition(rightFront.getCurrentPosition() + encoderChange);
        this.leftRear.setTargetPosition(leftRear.getCurrentPosition() + encoderChange);
        this.rightRear.setTargetPosition(rightRear.getCurrentPosition() - encoderChange);

        this.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    private boolean withinRange(DcMotor... motors) {
        for (DcMotor motor: motors) {
            if (Math.abs(motor.getTargetPosition() - motor.getCurrentPosition()) <= WITHIN_RANGE) {
                return true;
            }
        }
        return false;
    }

    private boolean allWithinRange(DcMotor... motors) {
        for (DcMotor motor: motors) {
            if (Math.abs(motor.getTargetPosition() - motor.getCurrentPosition()) > WITHIN_RANGE) {
                return false;
            }
        }
        return true;
    }
    private boolean withinRange()  {
        return withinRange(leftFront, rightFront, leftRear, rightRear);
    }

    public boolean allWithinRange()  {
        if (allWithinRange(leftFront, rightFront, leftRear, rightRear)) {
            stop();
            return true;
        }
        return false;
    }

    /**
     * Check if the drive train is within the specified encoder count
     * @return - true if the drive train is within encoder tolerance
     */
    public boolean driveTrainWithinRange() {
        if (withinRange())
        {
            stop();
            return true;
        }
        else {
            return false;
        }
    }

    public void stop() {
        lastLeftFrontEncoderValue = leftFront.getCurrentPosition();
        lastLeftRearEncoderValue = leftRear.getCurrentPosition();
        lastRightFrontEncoderValue = rightFront.getCurrentPosition();
        lastRightRearEncoderValue = rightRear.getCurrentPosition();
        //Stop our motors
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        reset();
    }

    public void reset() {
        resetEncoders();
        this.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void resetEncoders() {
        this.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public String getStatus() {
        return String.format(Locale.getDefault(),
                "LF(%s):%.2f(%d>%d),RF:%.2f(%d>%d),LR:%.2f(%d>%d),RR:%.2f(%d>%d)",
            this.leftFront.getDirection().toString(),
            this.leftFront.getPower(), this.leftFront.getCurrentPosition(), this.leftFront.getTargetPosition(),
            this.rightFront.getPower(), this.rightFront.getCurrentPosition(), this.rightFront.getTargetPosition(),
            this.leftRear.getPower(), this.leftRear.getCurrentPosition(), this.leftRear.getTargetPosition(),
            this.rightRear.getPower(), this.rightRear.getCurrentPosition(), this.rightRear.getTargetPosition());
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param coefficient  Proportional Gain Coefficient
     * @return - desired steering force
     */
    public static double getSteer(double error, double coefficient) {
        return Range.clip(error * coefficient, -1, 1);
    }

    /**
     * Drive in the specified direction at the specified speed while rotating at the specified rotation
     * Direction is relative to the robot
     * @param direction - direction to drive
     * @param speed - speed at which to drive
     * @param rotation - how much to rotate while driving
     */
    public void drive(double direction, double speed, double rotation) {
        double sin = Math.sin(direction + Math.PI / 4.0);
        double cos = Math.cos(direction + Math.PI / 4.0);
        double max = Math.max(Math.abs(sin), Math.abs(cos));
        sin /= max;
        cos /= max;

        double v1 = speed * sin + rotation;
        double v2 = speed * cos - rotation;
        double v3 = speed * cos + rotation;
        double v4 = speed * sin - rotation;

        // Ensure that none of the values go over 1.0. If none of the provided values are
        // over 1.0, just scale by 1.0 and keep all values.
        double scale = max(1.0, v1, v2, v3, v4);
        if (scale > 1) {
            v1 /= scale;
            v2 /= scale;
            v3 /= scale;
            v4 /= scale;
        }

        setLeftFrontPower(v1);
        setRightFrontPower(v2);
        setLeftRearPower(v3);
        setRightRearPower(v4);
    }


    /// Maximum absolute value of some number of arguments
    public static double max(double... xs) {
        double ret = 0.0;
        for (double x : xs) {
            ret = Math.max(ret, Math.abs(x));
        }
        return ret;
    }
}
