package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.teamcode.game.Alliance;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.components.Intake;
import org.firstinspires.ftc.teamcode.robot.components.drivetrain.DriveTrain;

import java.util.Date;
import java.util.Locale;

/**
 * Created by Silver Titans on 10/12/17.
 */

public class WiggleUntilFreightOperation extends Operation {
    /*
    Operation to wiggle a maximum number of times until a freight is consumed.
     */
    int maxWiggles = 0;
    double speed;
    DriveTrain driveTrain;
    Intake intake;
    String title;
    int wigglesDone;
    Date timeFreightCameIn;
    boolean lastMoveWasClockwise = false;
    public WiggleUntilFreightOperation(int maxWiggles,
                                       double speed, DriveTrain driveTrain, Intake intake, String title) {
        this.maxWiggles = maxWiggles;
        this.speed = speed;
        this.driveTrain = driveTrain;
        this.intake = intake;
        this.title = title;
    }

    public String toString() {
        return String.format(Locale.getDefault(), "WiggleUntilFreight: %d@%.2f --%s",
                maxWiggles, speed,
                this.title);
    }

    @Override
    public void startOperation() {
        //make sure we are set for intake
        intake.setForIntake();
        //we create a turn operation so we can get the drivetrain to move the right way
        if (Match.getInstance().getAlliance() == Alliance.Color.BLUE) {
            TurnClockwiseOperation turnClockwiseOperation =
                    new TurnClockwiseOperation(6 * Field.MM_PER_INCH, speed, driveTrain, title);
            driveTrain.handleOperation(turnClockwiseOperation);
            lastMoveWasClockwise = true;
        }
        else {
            TurnAntiClockwiseOperation turnAntiClockwiseOperation =
                    new TurnAntiClockwiseOperation(6 * Field.MM_PER_INCH, speed, driveTrain, title);
            driveTrain.handleOperation(turnAntiClockwiseOperation);
            lastMoveWasClockwise = false;
        }
    }

    public boolean isComplete() {
        //if we have freight, we stop the movement and say we are done
        if (intake.haveFreight()) {
            driveTrain.stop();
            return true;
        }
        else {
            if (driveTrain.driveTrainWithinRange()) {
                driveTrain.stop();
                wigglesDone++;
                if (wigglesDone < maxWiggles) {
                    if (lastMoveWasClockwise) {
                        //we create a turn operation so we can get the drivetrain to move the right way
                        Match.log("Adding anticlockwise turn");
                        TurnClockwiseOperation turnAntiClockwiseOperation =
                                new TurnAntiClockwiseOperation(6*Field.MM_PER_INCH, speed, driveTrain, title);
                        driveTrain.handleOperation(turnAntiClockwiseOperation);
                        lastMoveWasClockwise = false;
                    }
                    else {
                        //we create a turn operation so we can get the drivetrain to move the right way
                        Match.log("Adding clockwise turn");
                        TurnClockwiseOperation turnClockwiseOperation =
                                new TurnClockwiseOperation(6 * Field.MM_PER_INCH, speed, driveTrain, title);
                        driveTrain.handleOperation(turnClockwiseOperation);
                        lastMoveWasClockwise = true;
                    }
                    return false;
                }
                else {
                    Match.log("Reached max wiggles of " + wigglesDone);
                    return true;
                }
            }
            return false;
        }
    }
    @Override
    public void abortOperation() {
        driveTrain.stop();
    }
}

