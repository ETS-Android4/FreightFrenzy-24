package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.components.drivetrain.MecanumDriveTrain;

import java.util.Locale;

/**
 * Created by Silver Titans on 10/12/17.
 */

public class BearingOperation extends Operation {
    public static final double MIN_SPEED = 0.2;
    protected double desiredBearing;
    private MecanumDriveTrain driveTrain;

    public BearingOperation(double desiredBearing, MecanumDriveTrain driveTrain, String title) {
        this.title = title;
        this.desiredBearing = desiredBearing;
        this.driveTrain = driveTrain;
    }

    public String toString() {
        return String.format(Locale.getDefault(),"Bearing: %.2f --%s",
                this.desiredBearing, this.title);
    }

    @Override
    public boolean isComplete() {
        // determine turn power based on +/- error
        double currentBearing = driveTrain.getExternalHeading();
        double error = AngleUnit.normalizeDegrees(desiredBearing - Math.toDegrees(currentBearing));

        if (Math.abs(error) <= MecanumDriveTrain.HEADING_THRESHOLD) {
            driveTrain.stop();
            Match.log(String.format(Locale.getDefault(),
                    "Gyroscopic bearing finished with error: %.2f", error));
            return true;
        }
        else {
            double rotation = Math.max(Math.abs(error/180 * MecanumDriveTrain.P_TURN_COEFF), MIN_SPEED);
            rotation *= -Math.signum(error);
            driveTrain.drive(0, 0, rotation);
            Match.log(String.format(Locale.getDefault(),
                    "Gyroscopic bearing rotation: %.2f, error: %.2f, vslam:%s",
                    rotation, error, driveTrain.getPoseEstimate()));
            return false;
        }
    }

    @Override
    public void startOperation() {
        this.driveTrain.handleOperation(this);
    }

    @Override
    public void abortOperation() {
        this.driveTrain.stop();
    }
}
