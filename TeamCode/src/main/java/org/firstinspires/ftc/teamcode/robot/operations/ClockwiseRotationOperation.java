package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.teamcode.robot.components.drivetrain.MecanumDriveTrain;

import java.util.Locale;

/**
 * Created by Silver Titans on 10/12/17.
 */

public class ClockwiseRotationOperation extends Operation {
    private double degrees;
    private double speed;
    MecanumDriveTrain driveTrain;

    public ClockwiseRotationOperation(double degrees, double speed, MecanumDriveTrain driveTrain, String title) {
        this.degrees = degrees;
        this.speed = speed;
        this.driveTrain = driveTrain;
        this.title = title;
    }

    public String toString() {
        return String.format(Locale.getDefault(), "Degrees: %.2f@%.2f --%s",
                this.degrees, this.speed,
                this.title);
    }

    public double getSpeed() {
        return this.speed;
    }

    public double getDegrees() {
        return this.degrees;
    }

    @Override
    public boolean isComplete() {
        if (driveTrain.driveTrainWithinRange()) {
            driveTrain.stop();
            return true;
        }
        return false;
    }

    @Override
    public void startOperation() {
        driveTrain.handleOperation(this);
    }

    @Override
    public void abortOperation() {
        driveTrain.stop();
    }
}

