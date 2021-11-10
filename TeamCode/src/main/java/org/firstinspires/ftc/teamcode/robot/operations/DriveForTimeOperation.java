package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.teamcode.robot.components.drivetrain.MecanumDriveTrain;

import java.util.Date;
import java.util.Locale;

/**
 * Created by Silver Titans on 10/12/17.
 */

public class DriveForTimeOperation extends Operation {
    private long time;
    private double robotRelativeHeading;
    private double speed;
    private MecanumDriveTrain driveTrain;

    public void setSpeed(double speed) {
        this.speed = speed;
    }

    public long getTime() {
        return time;
    }

    public DriveForTimeOperation(long time, double heading, double speed, MecanumDriveTrain driveTrain, String title) {
        this.time = time;
        this.robotRelativeHeading = heading;
        this.speed = speed;
        this.driveTrain = driveTrain;
        this.title = title;
    }

    public String toString() {
        return String.format(Locale.getDefault(), "DriveForTime: %d@%.2f --%s",
                this.time, this.robotRelativeHeading,
                this.title);
    }

    public boolean isComplete() {
        if (new Date().getTime() > (this.getStartTime().getTime() + getTime())) {
            driveTrain.stop();
            return true;
        } else {
            driveTrain.drive(this.robotRelativeHeading, this.getSpeed(), 0);
            return false;
        }
    }

    public double getSpeed() {
        return this.speed;
    }

    @Override
    public void startOperation() {

    }

    @Override
    public void abortOperation() {

    }
}

