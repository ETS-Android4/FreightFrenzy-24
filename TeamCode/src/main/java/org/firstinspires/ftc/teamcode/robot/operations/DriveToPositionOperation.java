package org.firstinspires.ftc.teamcode.robot.operations;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.robot.components.drivetrain.DriveTrain;

import java.util.Locale;

/**
 * Operation to reach a specified pose.
 *
 * We accomplish this by using the FollowTrajectory class and setting its trajectory on the
 * fly when the operation is started.
 */

public class DriveToPositionOperation extends FollowTrajectory {
    private Pose2d desiredPose;
    private boolean trajectoryStarted;


    public boolean isTrajectoryStarted() {
        return trajectoryStarted;
    }

    public void setTrajectory (Trajectory trajectory) {
        this.trajectory = trajectory;
    }
    public void setTrajectoryStarted(boolean trajectoryStarted) {
        this.trajectoryStarted = trajectoryStarted;
    }

    public DriveToPositionOperation(Pose2d desiredPose, DriveTrain driveTrain, String title) {
        super(null, driveTrain, title);
        this.desiredPose = desiredPose;
        this.title = title;
    }

    public Pose2d getDesiredPose() {
        return this.desiredPose;
    }

    public String toString() {
        return String.format(Locale.getDefault(), "DriveTo: %s --%s",
                this.desiredPose.toString(),
                this.title);
    }

    public boolean isComplete() {
        //wait for the trajectory to be created
        if (!trajectoryStarted) {
            return false;
        }
        return super.isComplete();
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

