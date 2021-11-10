package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.teamcode.robot.RobotConfig;
import org.firstinspires.ftc.teamcode.robot.components.CarouselSpinner;

import java.util.Date;
import java.util.Locale;

/**
 * Created by Silver Titans on 10/12/17.
 */

public class CarouselOperation extends Operation {
    CarouselSpinner spinner;
    boolean clockwise;

    public CarouselOperation(CarouselSpinner spinner, boolean clockwise, String title) {
        this.spinner = spinner;
        this.clockwise = clockwise;
        this.title = title;
    }

    public String toString() {
        return String.format(Locale.getDefault(), "CarouselSpinner: --%s",
                this.title);
    }

    public boolean isComplete() {
        if (new Date().getTime() - getStartTime().getTime() > RobotConfig.CAROUSEL_SPINNER_REQUIRED_TIME) {
            spinner.stop();
            return true;
        }
        return false;
    }

    @Override
    public void startOperation() {
         if (clockwise) {
             spinner.setSpeed(RobotConfig.MAX_CAROUSEL_SPEED);
         }
         else {
             spinner.setSpeed(-RobotConfig.MAX_CAROUSEL_SPEED);
        }
    }

    @Override
    public void abortOperation() {
        spinner.stop();
    }
}

