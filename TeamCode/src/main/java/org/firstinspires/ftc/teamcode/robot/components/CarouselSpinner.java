package org.firstinspires.ftc.teamcode.robot.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.RobotConfig;

import java.util.Locale;

public class CarouselSpinner {
    DcMotor motor;
    public CarouselSpinner(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotor.class, RobotConfig.CAROUSEL_MOTOR);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setSpeed(double speed) {
        this.motor.setPower(speed);//Math.max((Math.min(speed, RobotConfig.MAX_CAROUSEL_SPEED)), -RobotConfig.MAX_CAROUSEL_SPEED));
    }

    public void stop() {
        this.motor.setPower(0);
    }

    public String getStatus() {
        return String.format(Locale.getDefault(),"C:%d->%d@%.2f",
                this.motor.getCurrentPosition(),
                this.motor.getTargetPosition(),
                this.motor.getPower());
    }
}
