package org.firstinspires.ftc.teamcode.robot.components;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.RobotConfig;

public class LED {
    RevBlinkinLedDriver blinkinLedDriver;

    public LED(HardwareMap hardwareMap) {
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, RobotConfig.BLINKIN);
        stop();
    }

    public void setPattern(RevBlinkinLedDriver.BlinkinPattern pattern) {
        this.blinkinLedDriver.setPattern(pattern);
    }

    public void stop() {
        this.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
    }

}
