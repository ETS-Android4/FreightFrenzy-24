package org.firstinspires.ftc.teamcode.robot.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.RobotConfig;
import org.firstinspires.ftc.teamcode.robot.operations.OutputOperation;

import java.util.Locale;

public class OutPutter {
    public static final int WITHIN_REACH = 5;
    DcMotor shoulder;
    Servo elbow;
    Servo bucketLid;
    int encoderOffset = 0;
    boolean inIntakePosition;
    Object synchronizer = new Object();

    public OutPutter(HardwareMap hardwareMap) {
        shoulder = hardwareMap.get(DcMotor.class, RobotConfig.OUT_SHOULDER);
        shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        elbow = hardwareMap.get(Servo.class, RobotConfig.OUT_ELBOW);
        bucketLid = hardwareMap.get(Servo.class, RobotConfig.BUCKET_LID);

        assumeInitialPosition();
    }

    public void setInIntakePosition(boolean inIntakePosition) {
        synchronized (synchronizer) {
            this.inIntakePosition = inIntakePosition;
        }
    }

    public boolean isInIntakePosition() {
        synchronized (synchronizer) {
            return this.inIntakePosition;
        }
    }

    public void setShoulderPosition(int position) {
        this.shoulder.setTargetPosition(position+ encoderOffset);
        this.shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.shoulder.setPower(RobotConfig.OUT_SHOULDER_SPEED);
    }

    public void setElbowPosition(double position) {
        this.elbow.setPosition(position);
    }
    /**
     * Get the arm so it is in the initial position
     */
    public void assumeInitialPosition() {
        this.close();
        this.elbow.setPosition(RobotConfig.OUTPUT_ELBOW_INITIAL_POSITION);
        this.shoulder.setTargetPosition(RobotConfig.OUTPUT_SHOULDER_INITIAL_POSITION);
        this.shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.shoulder.setPower(RobotConfig.OUT_SHOULDER_SPEED);
    }

    public boolean withinReach() {
        return Math.abs(shoulder.getTargetPosition() - shoulder.getCurrentPosition()) < WITHIN_REACH;
    }

    public void raiseAtShoulder() {
        this.setShoulderPosition(shoulder.getCurrentPosition() - encoderOffset + RobotConfig.OUTPUT_SHOULDER_INCREMENT);
    }
    public void lowerAtShoulder() {
        this.setShoulderPosition(shoulder.getCurrentPosition() -encoderOffset - RobotConfig.OUTPUT_SHOULDER_INCREMENT);
    }

    public void retractForearm() {
        this.elbow.setPosition(elbow.getPosition() - RobotConfig.OUTPUT_ELBOW_INCREMENT);
    }
    public void extendForearm() {
        this.elbow.setPosition(elbow.getPosition() + RobotConfig.OUTPUT_ELBOW_INCREMENT);
    }

    public void open() {
        this.bucketLid.setPosition(RobotConfig.OUTPUT_LID_OPEN_POSITION);
    }
    public void close() {
        this.bucketLid.setPosition(RobotConfig.OUTPUT_LID_CLOSED_POSITION);
    }
    public void closeLidMore() {
        this.bucketLid.setPosition(this.bucketLid.getPosition() + RobotConfig.OUTPUT_LID_INCREMENT);
    }
    public void openLidMore() {
        this.bucketLid.setPosition(this.bucketLid.getPosition() - RobotConfig.OUTPUT_LID_INCREMENT);
    }
    public void stop() {
    }

    public String getStatus() {
        return String.format(Locale.getDefault(),"Shoulder:%d->%d@%.2f(Offset:%d),Elbow:%.3f,Lid:%.3f",
                this.shoulder.getCurrentPosition(),
                this.shoulder.getTargetPosition(),
                this.shoulder.getPower(),
                this.encoderOffset,
                this.elbow.getPosition(),
                this.bucketLid.getPosition());
    }

    public void lidCappingPosition() {
        this.bucketLid.setPosition(RobotConfig.OUTPUT_LID_CAPPING_POSITION);
    }

    /**
     * To handle gear slippage, we allow for the driver to tell us where the current encoder value is
     * pointing.
     * @param type - the level at which the driver says we are right now
     */
    public void setEncoderOffset(OutputOperation.Type type) {
        switch (type) {
            case Level_Intake: {
                this.encoderOffset = this.shoulder.getCurrentPosition() - RobotConfig.OUTPUT_SHOULDER_INTAKE_POSITION;
                break;
            }
            case Level_Low: {
                this.encoderOffset = this.shoulder.getCurrentPosition() - RobotConfig.OUTPUT_SHOULDER_BOTTOM_POSITION;
                break;
            }
            case Level_Middle: {
                this.encoderOffset = this.shoulder.getCurrentPosition() - RobotConfig.OUTPUT_SHOULDER_MIDDLE_POSITION;
                break;
            }
            case Level_High: {
                this.encoderOffset = this.shoulder.getCurrentPosition() - RobotConfig.OUTPUT_SHOULDER_TOP_POSITION;
                break;
            }
        }
    }
}
