package org.firstinspires.ftc.teamcode.robot.components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.RobotConfig;

import java.util.Locale;

public class CappingArm {
    DcMotor motor;
    CRServo servo;
    public CappingArm(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotor.class, RobotConfig.ARM_MOTOR);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        servo = hardwareMap.get(CRServo.class, RobotConfig.ARM_SERVO);
        //keep servo from moving
        servo.setPower(0);
    }

    public void raiseArm() {
        this.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.motor.setTargetPosition(this.motor.getCurrentPosition() + RobotConfig.ARM_MOTOR_INCREMENT);
        this.motor.setPower(1);
    }

    public void lowerArm() {
        this.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.motor.setTargetPosition(this.motor.getCurrentPosition() - RobotConfig.ARM_MOTOR_INCREMENT);
        this.motor.setPower(1);
    }


    public void windServo() {
        this.servo.setPower(this.servo.getPower() + RobotConfig.ARM_SERVO_INCREMENT);
    }

    public void unwindServo() {
        this.servo.setPower(this.servo.getPower() - RobotConfig.ARM_SERVO_INCREMENT);
    }

    public void stopServo() {
        this.servo.setPower(0);
    }

    public void stop() {
        this.motor.setPower(0);
        this.servo.setPower(0);
    }

    public void setSpeed(double speed) {
        this.motor.setPower(speed);
    }


    public String getStatus() {
        return String.format(Locale.getDefault(), "A:%d->%d@%.2f,S:%.3f",
                this.motor.getCurrentPosition(),
                this.motor.getTargetPosition(),
                this.motor.getPower(),
                this.servo.getPower());
    }
}
