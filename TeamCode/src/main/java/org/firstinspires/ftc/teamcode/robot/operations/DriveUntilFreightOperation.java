package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.robot.RobotConfig;
import org.firstinspires.ftc.teamcode.robot.components.Intake;
import org.firstinspires.ftc.teamcode.robot.components.drivetrain.DriveTrain;

import java.util.Date;
import java.util.Locale;

/**
 * This operation drives a maximum of the specified distance in the heading provided at the speed
 * specified. It stops when the intake consumes a freight or the specified distance is traveled.
 */

public class DriveUntilFreightOperation extends DriveInDirectionOperation {
    Intake intake;
    Date timeFreightCameIn = null;
    /*
    Operation to travel a maximum distance until a freight is consumed.
    Operation is considered complete the moment freight is taken in, regardless of distance traveled
     */
    public DriveUntilFreightOperation(double maxTravelDistance, double heading,
                                      double speed, DriveTrain driveTrain, Intake intake, String title) {
        super(maxTravelDistance, heading, speed, driveTrain, title);
        this.intake = intake;
        this.title = title;
    }

    public String toString() {
        return String.format(Locale.getDefault(), "StraightLineUntilFreight: %.2f(%.2f\")@%.2f --%s",
                distance, (this.distance / Field.MM_PER_INCH), this.direction,
                this.title);
    }

    @Override
    public void startOperation() {
        //start driving in the direction specified
        super.startOperation();
        //make sure we are set for intake
        intake.setForIntake();
    }

    public boolean isComplete() {
        //if we have freight, we stop the movement and say we are done after giving enough time
        // for the freight to get to the bucket
        if (intake.haveFreight()) {
            if (timeFreightCameIn == null) {
                driveTrain.stop();
                Match.log(this.title + ": freight ingested, encoder ticks= " + driveTrain.getLastRightFrontEncoderValue());
                Match.getInstance().setDistanceTraveledForFreight(DriveConstants.encoderTicksToMMs(driveTrain.getLastRightFrontEncoderValue()));
                timeFreightCameIn = new Date();
                //even though the freight is in, we are not done as it probably has not reached
                //the bucket yet
                return false;
            }
            else {
                //provide enough time for freight to reach bucket
                return new Date().getTime() - timeFreightCameIn.getTime() > RobotConfig.SERVO_REQUIRED_TIME;
            }
        }
        else {
            //if freight hasn't come in, we go only as far the max distance
            if (super.isComplete()) {
                Match.log(this.title + ": freight NOT ingested, encoder ticks= " + driveTrain.getLastRightFrontEncoderValue());
                Match.getInstance().setDistanceTraveledForFreight(DriveConstants.encoderTicksToMMs(driveTrain.getLastRightFrontEncoderValue()));
                return true;
            }
            else {
                return false;
            }
        }
    }
}

