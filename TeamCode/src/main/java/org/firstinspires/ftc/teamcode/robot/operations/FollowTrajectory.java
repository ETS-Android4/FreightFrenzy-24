package org.firstinspires.ftc.teamcode.robot.operations;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.components.drivetrain.DriveTrain;

import java.util.Locale;

/**
 * An operation to follow the specified road-runner trajectory
 */

public class FollowTrajectory extends Operation {
    public static final int DEFAULT_CORRECTION_COUNT = 1;

    protected Trajectory trajectory;
    DriveTrain driveTrain;

    public FollowTrajectory(Trajectory trajectory, DriveTrain driveTrain, String title) {
        this.trajectory = trajectory;
        this.driveTrain = driveTrain;
        this.title = title;
    }
    public void setTrajectory(Trajectory trajectory) {
        this.trajectory = trajectory;
    }

    public String toString() {
        return String.format(Locale.getDefault(), "Trajectory: %s->%s --%s",
                this.trajectory.start().toString(),
                this.trajectory.end().toString(),
                this.title);
    }

    public boolean isComplete() {
        driveTrain.update();
        Pose2d currentPose = driveTrain.getPoseEstimate();
        String error = getError(currentPose, trajectory);
        Match.getInstance().setTrajectoryError(error);
        boolean busy = driveTrain.isBusy();
        if (!busy) {
            Match.log(String.format("Finished trajectory %s with error %s, at %s",
                    this.title,
                    error,
                    currentPose.toString()));
            return true;
        }
        else {
            //Match.log(error);
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

    public static String getError(Pose2d currentPose, Trajectory trajectory) {
        return String.format(Locale.getDefault(), "%.2f, %.2f, %.2f",
                        trajectory.end().getX()-currentPose.getX(),
                        trajectory.end().getY()-currentPose.getY(),
                        Math.toDegrees(
                                AngleUnit.normalizeRadians(
                                        trajectory.end().getHeading()
                                        - currentPose.getHeading()
                                )
                        )
        );
    }

    public Trajectory getTrajectory() {
        return trajectory;
    }
}

