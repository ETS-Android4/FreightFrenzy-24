package org.firstinspires.ftc.teamcode.robot.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.RobotConfig;

public class Carousel {
    DcMotor motor;
    public Carousel(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotor.class, RobotConfig.CAROUSEL_MOTOR);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setSpeed(double speed) {
        this.motor.setPower(speed);
    }

    public void stop() {
        this.motor.setPower(0);
    }

    public String getStatus() {
        return String.format("C:%d->%d@%.2f",
                this.motor.getCurrentPosition(),
                this.motor.getTargetPosition(),
                this.motor.getPower());
    }
}
