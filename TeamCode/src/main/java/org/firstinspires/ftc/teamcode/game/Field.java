package org.firstinspires.ftc.teamcode.game;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.robot.RobotConfig;

import static org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive.trajectoryBuilder;

/**
 * Created by Silver Titans on 9/16/17.
 */

public class Field {
    public static final float MM_PER_INCH = 25.4f;
    public static final double M_PER_INCH = MM_PER_INCH/1000;
    //the ROBOT_WIDTH of each tile
    public static final double TILE_ROBOT_WIDTH = 24 * MM_PER_INCH;
    // the ROBOT_WIDTH of the FTC field (from the center point to the outer panels)
    public static final double FIELD_ROBOT_WIDTH = 6*TILE_ROBOT_WIDTH;
    public static final double TAPE_ROBOT_WIDTH = 2*MM_PER_INCH;

    public static volatile boolean initialized = false;
    public static Object mutex = new Object();


    public enum StartingPosition {
        Left, Right
    }
    public enum Level {
        Bottom, Middle, High
    }
    public static final Pose2d[][] startingPoses = new Pose2d[Alliance.Color.values().length][StartingPosition.values().length];
    {
        startingPoses[Alliance.Color.RED.ordinal()][StartingPosition.Left.ordinal()] =
                new Pose2d(
                        (-2*Field.TILE_ROBOT_WIDTH + RobotConfig.ROBOT_WIDTH/2)/MM_PER_INCH,
                        (-Field.FIELD_ROBOT_WIDTH/2 + RobotConfig.ROBOT_LENGTH/2)/MM_PER_INCH,
                        Math.toRadians(90));
        startingPoses[Alliance.Color.RED.ordinal()][StartingPosition.Right.ordinal()] =
                new Pose2d(
                        (RobotConfig.ROBOT_WIDTH/2)/MM_PER_INCH,
                        (-Field.FIELD_ROBOT_WIDTH/2 + RobotConfig.ROBOT_LENGTH/2)/MM_PER_INCH,
                        Math.toRadians(90));
        startingPoses[Alliance.Color.BLUE.ordinal()][StartingPosition.Left.ordinal()] =
                new Pose2d(
                        (RobotConfig.ROBOT_WIDTH/2)/MM_PER_INCH,
                        (Field.FIELD_ROBOT_WIDTH/2-RobotConfig.ROBOT_LENGTH/2)/MM_PER_INCH,
                        Math.toRadians(-90));
        startingPoses[Alliance.Color.BLUE.ordinal()][StartingPosition.Right.ordinal()] =
                new Pose2d(
                        (-2*Field.TILE_ROBOT_WIDTH + RobotConfig.ROBOT_WIDTH/2)/MM_PER_INCH,
                        (Field.FIELD_ROBOT_WIDTH/2-RobotConfig.ROBOT_LENGTH/2)/MM_PER_INCH,
                        Math.toRadians(-90));
    }
    public static final Pose2d[][] hubDeliveryPoses = new Pose2d[Alliance.Color.values().length][StartingPosition.values().length];
    {
        hubDeliveryPoses[Alliance.Color.RED.ordinal()][StartingPosition.Left.ordinal()] =
                new Pose2d(
                        -42.5,
                        -24.0,
                        Math.toRadians(180));
        hubDeliveryPoses[Alliance.Color.RED.ordinal()][StartingPosition.Right.ordinal()] =
                new Pose2d(
                        0,
                        -2*Field.TILE_ROBOT_WIDTH/MM_PER_INCH,
                        Math.toRadians(135));
        hubDeliveryPoses[Alliance.Color.BLUE.ordinal()][StartingPosition.Left.ordinal()] =
                new Pose2d(
                        0,
                        2*Field.TILE_ROBOT_WIDTH/MM_PER_INCH,
                        Math.toRadians(-135));
        hubDeliveryPoses[Alliance.Color.BLUE.ordinal()][StartingPosition.Right.ordinal()] =
                new Pose2d(
                        -42.5,
                        24,
                        Math.toRadians(0));
    }
    public static final Pose2d[] spinningPoses = new Pose2d[Alliance.Color.values().length];
    {
        spinningPoses[Alliance.Color.RED.ordinal()] =
                new Pose2d(
                        -56,
                        -56,
                        Math.toRadians(225));
        spinningPoses[Alliance.Color.BLUE.ordinal()] =
                new Pose2d(
                        -56,
                        56,
                        Math.toRadians(135));
    }
    public static Trajectory reachHubTrajectory;
    public static Trajectory reachCarouselTrajectory;
    public static Trajectory navigateTrajectory;

    private Pose2d startingPose;

    public void init(Alliance.Color alliance, StartingPosition startingPosition) {
        startingPose = startingPoses[alliance.ordinal()][startingPosition.ordinal()];
        Thread initThread = new Thread(() -> {
            Match.log("Field initialization started for "
                    + alliance.toString() + ", " + startingPosition.toString());
            if (alliance == Alliance.Color.RED) {
                if (startingPosition == StartingPosition.Left) {
                    reachCarouselTrajectory =
                            trajectoryBuilder(startingPoses[alliance.ordinal()][startingPosition.ordinal()])
                                    .splineToLinearHeading(spinningPoses[alliance.ordinal()], 0).build();
                    Thread.yield();
                    Match.log("Created reach carousel trajectory");
                    reachHubTrajectory =
                            trajectoryBuilder(reachCarouselTrajectory.end(), true)
                                    .splineTo(new Vector2d(-54.0, -24.0), Math.toRadians(0.0))
                                    .splineTo(new Vector2d(-41.5, -24.0), 0)
                                    .build();
                    Thread.yield();
                    Match.log("Created reach hub trajectory");
                    navigateTrajectory = trajectoryBuilder(reachHubTrajectory.end())
                            .splineToConstantHeading(new Vector2d(-48.0, -24.0), Math.toRadians(180.0))
                            .splineToConstantHeading(new Vector2d(-60.0, -35.5), Math.toRadians(180.0))
                            .build();
                    Thread.yield();
                    Match.log("Created navigation trajectory");
                }
                else { //Red Right
                    reachHubTrajectory =
                            trajectoryBuilder(startingPoses[alliance.ordinal()][startingPosition.ordinal()])
                                    .splineTo(new Vector2d(0.0, -55.0), Math.toRadians(-90.0))
                                    .splineToLinearHeading(new Pose2d(-12.0, -44.0, Math.toRadians(-90.0)), 0.0)
                                    .build();
                    Thread.yield();
                    Match.log("Created reach hub trajectory");
                    navigateTrajectory = trajectoryBuilder(reachHubTrajectory.end())
                            .splineTo(reachHubTrajectory.end().vec().minus(new Vector2d(0.0, 9.0)), reachHubTrajectory.end().getHeading())
                            .splineTo(new Vector2d(12.0, -48.0), Math.toRadians(0.0))
                            .build();
                    Thread.yield();
                    Match.log("Created navigation trajectory");
                }
            }
            if (alliance == Alliance.Color.BLUE) {
                if (startingPosition == StartingPosition.Right) {
                    reachCarouselTrajectory =
                            trajectoryBuilder(startingPoses[alliance.ordinal()][startingPosition.ordinal()])
                                    .splineToLinearHeading(spinningPoses[alliance.ordinal()], 0).build();
                    Thread.yield();
                    Match.log("Created reach carousel trajectory");
                    reachHubTrajectory =
                            trajectoryBuilder(reachCarouselTrajectory.end(), true)
                                    .splineTo(new Vector2d(-54.0, 24.0), Math.toRadians(0.0))
                                    .splineTo(new Vector2d(-41.5, 24.0), 0)
                                    .build();
                    Thread.yield();
                    Match.log("Created reach hub trajectory");
                    navigateTrajectory = trajectoryBuilder(reachHubTrajectory.end())
                            .splineToConstantHeading(new Vector2d(-48.0, 24.0), Math.toRadians(180.0))
                            .splineToConstantHeading(new Vector2d(-60.0, 35.5), Math.toRadians(180.0))
                            .build();
                    Thread.yield();
                    Match.log("Created navigation trajectory");
                }
                else { //Blue Left
                    reachHubTrajectory =
                            trajectoryBuilder(startingPoses[alliance.ordinal()][startingPosition.ordinal()])
                                    .splineTo(new Vector2d(0.0, 55.0), Math.toRadians(90.0))
                                    .splineToLinearHeading(new Pose2d(-12.0, 44.0, Math.toRadians(90.0)), 0.0)
                                    .build();
                    Thread.yield();
                    Match.log("Created reach hub trajectory");
                    navigateTrajectory = trajectoryBuilder(reachHubTrajectory.end())
                            .splineTo(reachHubTrajectory.end().vec().plus(new Vector2d(0.0, 9.0)), reachHubTrajectory.end().getHeading())
                            .splineTo(new Vector2d(12.0, 48.0), Math.toRadians(0.0))
                            .build();
                    Thread.yield();
                    Match.log("Created navigation trajectory");
                }
            }

            synchronized (mutex) {
                initialized = true;
            }
        });
        initThread.start();
    }

    public static boolean isInitialized() {
        synchronized (mutex) {
            return initialized;
        }
    }

    /**
     * Convert from the poses used by the RoadRunner library to those used by the T265 Camera.
     * The units of measure of the x and y coordinates in the T265 Camera are meters
     * while those of RoadRunner library are inches. The headings are in radians in both cases.
     *
     * @param roadRunnerPose
     * @return
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
     * @param cameraPose
     * @return
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