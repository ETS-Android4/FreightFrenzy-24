package org.firstinspires.ftc.teamcode.robot.components;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.RobotConfig;

import java.util.Locale;

public class Intake {
    DcMotor motor;
    Servo servo;
    ColorSensor colorSensor;
    DistanceSensor distanceSensor;
    Object synchronizer = new Object();
    boolean keepRunning = true;
    boolean interruptIntake = false;
    boolean haveFreight = false;
    OutPutter outPutter;

    int red = 0, green = 0, blue = 0;
    double distance;
    public Intake(HardwareMap hardwareMap, OutPutter outPutter) {
        this.outPutter = outPutter;
        motor = hardwareMap.get(DcMotor.class, RobotConfig.IN_MOTOR);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        servo = hardwareMap.get(Servo.class, RobotConfig.IN_SERVO);
        servo.setPosition(RobotConfig.INTAKE_LOWERED_POSITION);

        colorSensor = hardwareMap.get(ColorSensor.class, RobotConfig.IN_SENSOR);
        distanceSensor = hardwareMap.get(DistanceSensor.class, RobotConfig.IN_SENSOR);
        Thread initThread = new Thread(() -> {
            while (keepRunning()) {
                red = 0;
                if (interruptIntake())
                    try {
                        red = colorSensor.red();
                        green = colorSensor.green();
                        blue = colorSensor.blue();
                        distanceSensor.getDistance(DistanceUnit.MM);
                    }
                    catch (Throwable e) {
                        Match.log("Error: " + e.toString());
                    }
                    if (red > 140) {
                        Match.log("Interrupted for " + red);
                        setSpeed(0);
                        setHaveFreight(true);
                        if (outPutter.isInIntakePosition()) {
                            servo.setPosition(RobotConfig.INTAKE_RAISED_POSITION);
                        }
                        else {
                            servo.setPosition(RobotConfig.INTAKE_CONSUME_POSITION);
                        }
                        setInterruption(false);
                    }
                }
                Thread.yield();
        });
        initThread.start();
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
        this.setSpeed(RobotConfig.INTAKE_SPEED);
        setHaveFreight(false);
        //ensure we only get one freight in
        this.setInterruption(true);
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

    public boolean haveFreight() {
        synchronized (synchronizer) {
            return haveFreight;
        }
    }

    public void setHaveFreight(boolean haveFreight) {
        synchronized (synchronizer) {
            this.haveFreight = haveFreight;
            if (haveFreight) {
                Match.getInstance().setLed(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            }
            else {
                Match.getInstance().setLed(RevBlinkinLedDriver.BlinkinPattern.WHITE);
            }
        }
    }

    public String getStatus() {
        return String.format(Locale.getDefault(),"M:%.2f,S:%.3f,RGB:%d;%d;%d,Dist:%.2f",
                this.motor.getPower(),
                servo.getPosition(),
                red,
                green,
                blue,
                distance);
    }

    @Override
    public void finalize() {
        stopRunning();
    }
}
