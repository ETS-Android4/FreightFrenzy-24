package org.firstinspires.ftc.teamcode.game;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.robot.RobotConfig;

import static org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive.accurateTrajectoryBuilder;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive.trajectoryBuilder;

/**
 * Created by Silver Titans on 9/16/17.
 */

/*
       Top y position: -49
       Middle y position: -53
       Bottom y position: -51
 */
public class Field {
    public static final float MM_PER_INCH = 25.4f;
    public static final double M_PER_INCH = MM_PER_INCH/1000;
    public static final double TOP_Y_POSITION = 49;
    public static final double MIDDLE_Y_POSITION = 52;
    public static final double BOTTOM_Y_POSITION = 49;
    //the ROBOT_WIDTH of each tile
    public static final double TILE_WIDTH = 24 * MM_PER_INCH;
    // the ROBOT_WIDTH of the FTC field (from the center point to the outer panels)
    public static final double FIELD_WIDTH = 6* TILE_WIDTH;
    public static final double CHANNEL_CENTER = 67;

    public static volatile boolean initialized = false;
    public static final Object mutex = new Object();


    public enum StartingPosition {
        Left, Right
    }

    public static final Pose2d[][] startingPoses = new Pose2d[Alliance.Color.values().length][StartingPosition.values().length];
    static {
        startingPoses[Alliance.Color.RED.ordinal()][StartingPosition.Left.ordinal()] =
                new Pose2d(
                        (-2*Field.TILE_WIDTH + RobotConfig.ROBOT_WIDTH/2)/MM_PER_INCH,
                        (-Field.FIELD_WIDTH/2 + RobotConfig.ROBOT_CENTER_FROM_BACK)/MM_PER_INCH,
                        Math.toRadians(90));
        startingPoses[Alliance.Color.RED.ordinal()][StartingPosition.Right.ordinal()] =
                new Pose2d(
                        (RobotConfig.ROBOT_WIDTH/2)/MM_PER_INCH,
                        (-Field.FIELD_WIDTH/2 + RobotConfig.ROBOT_CENTER_FROM_BACK)/MM_PER_INCH,
                        Math.toRadians(90));
        startingPoses[Alliance.Color.BLUE.ordinal()][StartingPosition.Left.ordinal()] =
                new Pose2d(
                        (RobotConfig.ROBOT_WIDTH/2)/MM_PER_INCH,
                        (Field.FIELD_WIDTH/2 -RobotConfig.ROBOT_CENTER_FROM_BACK)/MM_PER_INCH,
                        Math.toRadians(-90));
        startingPoses[Alliance.Color.BLUE.ordinal()][StartingPosition.Right.ordinal()] =
                new Pose2d(
                        (-2*Field.TILE_WIDTH + RobotConfig.ROBOT_WIDTH/2)/MM_PER_INCH,
                        (Field.FIELD_WIDTH /2 - RobotConfig.ROBOT_CENTER_FROM_BACK)/MM_PER_INCH,
                        Math.toRadians(-90));
    }
    public static final Pose2d[] spinningPoses = new Pose2d[Alliance.Color.values().length];
    static {
        spinningPoses[Alliance.Color.RED.ordinal()] =
                new Pose2d(
                        -58,
                        -58,
                        Math.toRadians(225));
        spinningPoses[Alliance.Color.BLUE.ordinal()] =
                new Pose2d(
                        -58,
                        58,
                        Math.toRadians(135));
    }
    public static Trajectory reachHubTrajectory;
    public static Trajectory reachCarouselTrajectory;
    public static Trajectory navigateTrajectory;
    public static Trajectory reachHubAgainTrajectory;

    private Pose2d startingPose;

    public void init(Alliance.Color alliance, StartingPosition startingPosition) {
        startingPose = startingPoses[alliance.ordinal()][startingPosition.ordinal()];
        boolean nearCarousel = (alliance == Alliance.Color.RED && startingPosition == StartingPosition.Left)
                || (alliance == Alliance.Color.BLUE && startingPosition == StartingPosition.Right);
        Thread initThread = new Thread(() -> {
            Match.log("Field initialization started for "
                    + alliance.toString() + ", " + startingPosition.toString());
            double yMultiplier = alliance == Alliance.Color.RED ? -1 : 1;
            if (nearCarousel) {
                reachCarouselTrajectory =
                        trajectoryBuilder(startingPose)
                                .splineTo(spinningPoses[alliance.ordinal()].vec(), Math.toRadians(-225.0*yMultiplier)).build();
                Thread.yield();
                Match.log("Created reach carousel trajectory");
                reachHubTrajectory =
                        trajectoryBuilder(reachCarouselTrajectory.end(), true)
                                //.splineTo(new Vector2d(-12.0, 52.0 * yMultiplier), Math.toRadians(-90.0 * yMultiplier))
                                .splineTo(new Vector2d(-12.0, MIDDLE_Y_POSITION * yMultiplier), Math.toRadians(-90.0 * yMultiplier))
                                .build();
                Thread.yield();
                Match.log("Created reach hub trajectory");
                navigateTrajectory = accurateTrajectoryBuilder(reachHubTrajectory.end())
                        //.splineTo(new Vector2d(0.0, (CHANNEL_CENTER-5) * yMultiplier), Math.toRadians(0.0))
                        .splineTo(new Vector2d(16.0, (CHANNEL_CENTER-3) * yMultiplier), Math.toRadians(0))
                        //.splineTo(new Vector2d(32.0, CHANNEL_CENTER * yMultiplier), Math.toRadians(0))
                        .splineTo(new Vector2d(48.0, CHANNEL_CENTER * yMultiplier), Math.toRadians(0))
                        .build();
                Thread.yield();
                reachHubAgainTrajectory = accurateTrajectoryBuilder(navigateTrajectory.end(), true)
                        .splineTo(new Vector2d(12.0, CHANNEL_CENTER * yMultiplier), Math.toRadians(-180.0 * yMultiplier))
                        .splineTo(new Vector2d(-12.0, TOP_Y_POSITION * yMultiplier), Math.toRadians(-90.0 * yMultiplier))
                        .build();

            } else { // Near warehouse
                reachCarouselTrajectory =
                        trajectoryBuilder(startingPose)
                                //.splineTo(new Vector2d(-24.0, 58.0 * yMultiplier), Math.toRadians(180.0 * yMultiplier))
                                .splineTo(new Vector2d(-58.0, 58.0 * yMultiplier), Math.toRadians(-225.0 * yMultiplier))
                        .build();
                Thread.yield();
                Match.log("Created reach carousel trajectory");
                reachHubTrajectory =
                        trajectoryBuilder(reachCarouselTrajectory.end(), true)
                                .splineTo(new Vector2d(-12.0, 52.0 * yMultiplier), Math.toRadians(-90.0 * yMultiplier))
                                .splineTo(new Vector2d(-12.0, MIDDLE_Y_POSITION * yMultiplier), Math.toRadians(-90.0 * yMultiplier))
                                .build();
                Thread.yield();
                Match.log("Created reach hub trajectory");
                navigateTrajectory = trajectoryBuilder(reachHubTrajectory.end())
                        .splineTo(new Vector2d(12.0, 60.0 * yMultiplier), Math.toRadians(0))
                        .splineTo(new Vector2d(48.0, 62.0 * yMultiplier), Math.toRadians(0))
                        .build();
                Thread.yield();
                reachHubAgainTrajectory = trajectoryBuilder(navigateTrajectory.end(), true)
                        .splineTo(new Vector2d(12.0, 62.0 * yMultiplier), Math.toRadians(-180.0 * yMultiplier))
                        .splineTo(new Vector2d(-12.0, 48.0 * yMultiplier), Math.toRadians(-90.0 * yMultiplier))
                        .build();
                Thread.yield();
                Match.log("Created navigation trajectory");
            }

            synchronized (mutex) {
                initialized = true;
            }
        });
        initThread.start();
    }

    public static boolean isNotInitialized() {
        synchronized (mutex) {
            return !initialized;
        }
    }

    /**
     * Convert from the poses used by the RoadRunner library to those used by the T265 Camera.
     * The units of measure of the x and y coordinates in the T265 Camera are meters
     * while those of RoadRunner library are inches. The headings are in radians in both cases.
     *
     * @param roadRunnerPose - pose in inches and radians
     * @return pose in meters and radians
     */
    public static com.arcrobotics.ftclib.geometry.Pose2d roadRunnerToCameraPose(Pose2d roadRunnerPose) {
        return new com.arcrobotics.ftclib.geometry.Pose2d
                    (roadRunnerPose.getX()*M_PER_INCH, roadRunnerPose.getY()*M_PER_INCH,
                new Rotation2d(roadRunnerPose.getHeading()));
    }

    /**
     * Convert from the poses used by the T265 Camera and those used by the RoadRunner library.
     * The units of measure of the x and y coordinates in the T265 Camera are meters
     * while those of RoadRunner library are inches. The headings are in radians in both cases.
     *
     * @param cameraPose - pose in meters
     * @return pose in inches
     */
    public static Pose2d cameraToRoadRunnerPose(com.arcrobotics.ftclib.geometry.Pose2d cameraPose) {
        return new Pose2d(
                    cameraPose.getTranslation().getX()/M_PER_INCH,
                    cameraPose.getTranslation().getY()/M_PER_INCH,
                cameraPose.getHeading());
    }

    public Pose2d getStartingPose() {
        return startingPose;
    }
}