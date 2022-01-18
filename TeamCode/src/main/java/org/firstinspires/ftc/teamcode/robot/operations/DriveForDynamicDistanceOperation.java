package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.components.drivetrain.DriveTrain;

import java.util.Locale;

/**
 * Created by Silver Titans on 10/12/17.
 */

public class DriveForDynamicDistanceOperation extends DriveInDirectionOperation {
    boolean reversed;
    double dynamicDistance;
    public DriveForDynamicDistanceOperation(boolean reverse, double heading, double speed, DriveTrain driveTrain, String title) {
        super(0, heading, speed, driveTrain, title);
        this.reversed = reverse;
    }

    public String toString() {
        return String.format(Locale.getDefault(), "DynamicDistance: %.2f(%.2f\")@%.2f --%s",
                this.dynamicDistance, this.dynamicDistance/ Field.MM_PER_INCH, this.speed,
                this.title);
    }


    @Override
    public void startOperation() {
        this.dynamicDistance = Match.getInstance().getDistanceTraveledForFreight() * (reversed ? -1 : 1);
        Match.log("Set dynamic distance to " + dynamicDistance);
        super.setDistance(dynamicDistance);
        super.startOperation();
    }

    @Override
    public void abortOperation() {
        driveTrain.stop();
    }
}

