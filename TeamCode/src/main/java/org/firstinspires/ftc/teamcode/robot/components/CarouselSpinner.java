package org.firstinspires.ftc.teamcode.robot.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.RobotConfig;

import java.util.Locale;

public class CarouselSpinner {
    //variable called motor
    private DcMotor motor;

    //constructor that requires HardwareMap to be passed in
    public CarouselSpinner(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotor.class, RobotConfig.CAROUSEL_MOTOR);
    }

    /**
     * Method to set the speed of the carousel spinner motor
     * @param speed
     */
    public void setSpeed(double speed) {
        double speedToSet =
                Math.max((Math.min(speed, RobotConfig.MAX_CAROUSEL_SPEED)),
                        -RobotConfig.MAX_CAROUSEL_SPEED);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motor.setPower(speedToSet);
        if (speedToSet != 0) {
            Match.log("Set carousel speed to " + speedToSet);
        }
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
