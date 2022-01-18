package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.robot.components.drivetrain.DriveTrain;

import java.util.Locale;

/**
 * Created by Silver Titans on 10/12/17.
 */

public class TurnClockwiseOperation extends DriveForDistanceOperation {

    public TurnClockwiseOperation(double distance, double speed, DriveTrain driveTrain, String title) {
        super(distance, speed, driveTrain, title);
    }

    public String toString() {
        return String.format(Locale.getDefault(), "TurnClockwise: %.2f(%.2f\")@%.2f --%s",
                this.distance, this.distance/ Field.MM_PER_INCH, this.speed,
                this.title);
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

