package org.firstinspires.ftc.teamcode.robot.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.RobotConfig;

import java.util.Locale;

public class Intake {
    DcMotor motor;
    Servo servo;
    public Intake(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotor.class, RobotConfig.IN_MOTOR);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        servo = hardwareMap.get(Servo.class, RobotConfig.IN_SERVO);
        servo.setPosition(RobotConfig.INTAKE_RAISED_POSITION);
    }

    public void setSpeed(double speed) {
        this.motor.setPower(Math.max((Math.min(speed, RobotConfig.MAX_IN_SPEED)), -RobotConfig.MAX_IN_SPEED));
    }

    public void setForIntake() {
        this.servo.setPosition(RobotConfig.INTAKE_LOWERED_POSITION);
    }

    public void setForOutput() {
        this.servo.setPosition(RobotConfig.INTAKE_RAISED_POSITION);
    }

    public void raiseIntake() {
        this.servo.setPosition(servo.getPosition() - RobotConfig.INTAKE_SERVO_INCREMENT);
    }

    public void lowerIntake() {
        this.servo.setPosition(servo.getPosition() + RobotConfig.INTAKE_SERVO_INCREMENT);
    }

    public void stop() {
        this.motor.setPower(0);
    }

    public String getStatus() {
        return String.format(Locale.getDefault(),"%d->%d@%.2f,Servo%.3f",
                this.motor.getCurrentPosition(),
                this.motor.getTargetPosition(),
                this.motor.getPower(),
                servo.getPosition());
    }
}
