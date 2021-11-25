package org.firstinspires.ftc.teamcode.robot.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.RobotConfig;

import java.util.Locale;

public class OutPutter {
    public static final int WITHIN_REACH = 5;
    int initialPosition = 0;
    DcMotor shoulder;
    Servo elbow;
    Servo gripper;

    public int getInitialPosition() {
        return initialPosition;
    }

    public void setInitialPosition() {
        this.initialPosition = shoulder.getCurrentPosition();
    }

    public OutPutter(HardwareMap hardwareMap) {
        shoulder = hardwareMap.get(DcMotor.class, RobotConfig.OUT_SHOULDER);
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        elbow = hardwareMap.get(Servo.class, RobotConfig.OUT_ELBOW);
        gripper = hardwareMap.get(Servo.class, RobotConfig.OUT_GRIPPER);

        fold();
    }

    public void setSpeed(double speed) {
        this.shoulder.setPower(Math.max((Math.min(speed, RobotConfig.MAX_IN_SPEED)), -RobotConfig.MAX_IN_SPEED));
    }
    public void setLowPosition() {
        this.elbow.setPosition(RobotConfig.OUT_ALIGNER_BOTTOM_POSITION);
    }
    public void setMiddlePosition() {
        this.elbow.setPosition(RobotConfig.OUT_ALIGNER_MIDDLE_POSITION);
    }
    public void setHighPosition() {
        this.elbow.setPosition(RobotConfig.OUT_ALIGNER_TOP_POSITION);
    }
    public void fold() {
        this.elbow.setPosition(RobotConfig.OUT_ALIGNER_FOLDED_POSITION);
    }

    public boolean withinReach() {
        return Math.abs(shoulder.getTargetPosition() - shoulder.getCurrentPosition()) < WITHIN_REACH;
    }

    public void deliver() {
        this.shoulder.setTargetPosition(RobotConfig.DELIVERY_ENCODER_VALUE + initialPosition);
        this.shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.shoulder.setPower(.4);
    }

    public void retract() {
        this.shoulder.setTargetPosition(RobotConfig.INTAKE_ENCODER_VALUE + initialPosition);
        this.shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.shoulder.setPower(.4);
    }
    public void raiseArm() {
        this.elbow.setPosition(elbow.getPosition() - RobotConfig.OUT_ALIGNER_SERVO_INCREMENT);
    }

    public void lowerArm() {
        this.elbow.setPosition(elbow.getPosition() + RobotConfig.OUT_ALIGNER_SERVO_INCREMENT);
    }
    public void stop() {
        this.shoulder.setPower(0);
        this.elbow.setPosition(RobotConfig.OUT_ALIGNER_FOLDED_POSITION);
    }

    public String getStatus() {
        return String.format(Locale.getDefault(),"%d->%d@%.2f,Servo:%.3f",
                this.shoulder.getCurrentPosition(),
                this.shoulder.getTargetPosition(),
                this.shoulder.getPower(),
                this.elbow.getPosition());
    }
}
