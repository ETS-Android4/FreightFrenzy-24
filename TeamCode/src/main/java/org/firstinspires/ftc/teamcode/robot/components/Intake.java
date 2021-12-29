package org.firstinspires.ftc.teamcode.robot.components;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.RobotConfig;

import java.util.Locale;

public class Intake {
    DcMotor motor;
    Servo servo;
    Rev2mDistanceSensor distanceSensor;
    Object synchronizer = new Object();
    boolean keepRunning = true;
    boolean interruptIntake = false;
    public Intake(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotor.class, RobotConfig.IN_MOTOR);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        servo = hardwareMap.get(Servo.class, RobotConfig.IN_SERVO);
        servo.setPosition(RobotConfig.INTAKE_LOWERED_POSITION);

        //distanceSensor = (Rev2mDistanceSensor) hardwareMap.get(DistanceSensor.class, RobotConfig.IN_SENSOR);
        Thread initThread = new Thread(() -> {
            while (keepRunning()) {
                if (interruptIntake()) {
                    if (distanceSensor.getDistance(DistanceUnit.MM) < 50) {
                        setSpeed(0);
                    }
                }
            }
        });
        //initThread.start();
    }

    public boolean keepRunning() {
        synchronized (synchronizer) {
            return keepRunning;
        }
    }
    public void stopRunning() {
        synchronized (synchronizer) {
            keepRunning = false;
        }
    }
    public boolean interruptIntake() {
        synchronized (synchronizer) {
            return interruptIntake;
        }
    }
    public void setInterruption(boolean interruptIntake) {
        synchronized (synchronizer) {
            this.interruptIntake = interruptIntake;
        }
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

    public void setForExpelling() {
        this.servo.setPosition(RobotConfig.INTAKE_LOWERED_POSITION);
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
        return String.format(Locale.getDefault(),"%d->%d@%.2f,Servo%.3f,Distance:%.2f",
                this.motor.getCurrentPosition(),
                this.motor.getTargetPosition(),
                this.motor.getPower(),
                servo.getPosition(),
                distanceSensor.getDistance(DistanceUnit.MM));
    }

    @Override
    public void finalize() {
        stopRunning();
    }
}
