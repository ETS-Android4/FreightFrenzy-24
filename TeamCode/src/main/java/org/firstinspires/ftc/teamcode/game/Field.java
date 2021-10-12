package org.firstinspires.ftc.teamcode.game;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.robot.RobotConfig;
import org.firstinspires.ftc.teamcode.robot.components.drivetrain.MecanumDriveTrain;

/**
 * Created by Silver Titans on 9/16/17.
 */

public class Field {
    public static final float MM_PER_INCH = 25.4f;
    public static final double M_PER_INCH = MM_PER_INCH/1000;
    //the width of each tile
    public static final double TILE_WIDTH = 24 * MM_PER_INCH;
    // the width of the FTC field (from the center point to the outer panels)
    public static final double FIELD_WIDTH = 6*TILE_WIDTH;
    public static final double TAPE_WIDTH = 2*MM_PER_INCH;

    public static volatile boolean initialized = false;
    public static Object mutex = new Object();


    public enum StartingPosition {
        LEFT, RIGHT
    }
    public static final double ROBOT_STARTING_X =
            -(Field.FIELD_WIDTH/2 - MecanumDriveTrain.DRIVE_TRAIN_LENGTH/2 - 2*Field.MM_PER_INCH);
    public static final Pose2d[][] startingPoses = new Pose2d[Alliance.Color.values().length][StartingPosition.values().length];
    {
        startingPoses[Alliance.Color.RED.ordinal()][StartingPosition.LEFT.ordinal()] =
                new Pose2d(
                        (-Field.TILE_WIDTH - RobotConfig.WIDTH/2)/MM_PER_INCH,
                        (-Field.FIELD_WIDTH/2+RobotConfig.LENGTH/2)/MM_PER_INCH,
                        Math.toRadians(90));
        startingPoses[Alliance.Color.RED.ordinal()][StartingPosition.RIGHT.ordinal()] =
                new Pose2d(
                        (RobotConfig.WIDTH/2)/MM_PER_INCH,
                        (-Field.FIELD_WIDTH/2+RobotConfig.LENGTH)/2/MM_PER_INCH,
                        Math.toRadians(90));
        startingPoses[Alliance.Color.BLUE.ordinal()][StartingPosition.LEFT.ordinal()] =
                new Pose2d(
                        (RobotConfig.WIDTH/2)/MM_PER_INCH,
                        (Field.FIELD_WIDTH/2-RobotConfig.LENGTH/2)/MM_PER_INCH,
                        Math.toRadians(-90));
        startingPoses[Alliance.Color.BLUE.ordinal()][StartingPosition.RIGHT.ordinal()] =
                new Pose2d(
                        (-Field.TILE_WIDTH - RobotConfig.WIDTH/2)/MM_PER_INCH,
                        (Field.FIELD_WIDTH/2-RobotConfig.LENGTH/2)/MM_PER_INCH,
                        Math.toRadians(-90));
    }
    private Pose2d startingPose;

    public void init(Alliance.Color alliance, StartingPosition startingPosition) {
        startingPose = startingPoses[alliance.ordinal()][startingPosition.ordinal()];
        Thread initThread = new Thread(() -> {
            RobotLog.i("SilverTitans: Field initialization started");
/*
            //Red-right first wobble trajectories
            dropFirstWobbleTrajectories[RingCount.NONE.ordinal()] =
                    new TrajectoryBuilder(
                            STARTING_POSE,
                            PhoebeRoadRunnerDrive.fastVelocityConstraint, PhoebeRoadRunnerDrive.accelerationConstraint)
                            .splineToConstantHeading(new Vector2d(-24, NONE_DEPOSIT_POSE_FIRST.getY()), 0)
                            .splineToConstantHeading(NONE_DEPOSIT_POSE_FIRST.vec(), 0)
                            .build();
            dropFirstWobbleTrajectories[RingCount.ONE.ordinal()] =
                    new TrajectoryBuilder(
                            STARTING_POSE, PhoebeRoadRunnerDrive.fastVelocityConstraint, PhoebeRoadRunnerDrive.accelerationConstraint)
                            .splineToConstantHeading(new Vector2d(-24, NONE_DEPOSIT_POSE_FIRST.getY()), 0)
                            .splineToConstantHeading(ONE_DEPOSIT_POSE_FIRST.vec(), 0)
                            .build();
            dropFirstWobbleTrajectories[RingCount.FOUR.ordinal()] =
                    new TrajectoryBuilder(
                             STARTING_POSE, PhoebeRoadRunnerDrive.fastVelocityConstraint, PhoebeRoadRunnerDrive.accelerationConstraint)
                            .splineToConstantHeading(new Vector2d(-24, NONE_DEPOSIT_POSE_FIRST.getY()), 0)
                            .splineToConstantHeading(FOUR_DEPOSIT_POSE_FIRST.vec(), 0)
                            .build();
            RobotLog.i("SilverTitans: Created drop first wobble trajectories");

 */
            synchronized (mutex) {
                initialized = true;
                RobotLog.i("SilverTitans: Field initialization completed");
            }
        });
        initThread.start();
    }

    public static boolean isInitialized() {
        synchronized (mutex) {
            return initialized;
        }
    }

    public void generateTrajectories() {
        Thread initThread = new Thread(() -> {
        });
        initThread.start();
    }

    public static com.arcrobotics.ftclib.geometry.Pose2d roadRunnerToCameraPose(Pose2d pose) {
        return new com.arcrobotics.ftclib.geometry.Pose2d(pose.getX()*M_PER_INCH, pose.getY()*M_PER_INCH,
                new Rotation2d(pose.getHeading()+Math.PI));
    }
    public static Pose2d cameraToRoadRunnerPose(com.arcrobotics.ftclib.geometry.Pose2d pose) {
        return new Pose2d(pose.getTranslation().getX()/M_PER_INCH, pose.getTranslation().getY()/M_PER_INCH,
                pose.getHeading()+Math.PI);
    }

    public Pose2d getStartingPose() {
        return startingPose;
    }
}