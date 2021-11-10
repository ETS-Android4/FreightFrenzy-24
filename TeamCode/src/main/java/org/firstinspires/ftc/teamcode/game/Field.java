package org.firstinspires.ftc.teamcode.game;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
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
    public static final Pose2d[][] startingPoses = new Pose2d[Alliance.Color.values().length][StartingPosition.values().length];
    {
        startingPoses[Alliance.Color.RED.ordinal()][StartingPosition.LEFT.ordinal()] =
                new Pose2d(
                        (-2*Field.TILE_WIDTH + RobotConfig.WIDTH/2)/MM_PER_INCH,
                        (-Field.FIELD_WIDTH/2 + RobotConfig.LENGTH/2)/MM_PER_INCH,
                        Math.toRadians(90));
        startingPoses[Alliance.Color.RED.ordinal()][StartingPosition.RIGHT.ordinal()] =
                new Pose2d(
                        (RobotConfig.WIDTH/2)/MM_PER_INCH,
                        (-Field.FIELD_WIDTH/2 + RobotConfig.LENGTH/2)/MM_PER_INCH,
                        Math.toRadians(90));
        startingPoses[Alliance.Color.BLUE.ordinal()][StartingPosition.LEFT.ordinal()] =
                new Pose2d(
                        (RobotConfig.WIDTH/2)/MM_PER_INCH,
                        (Field.FIELD_WIDTH/2-RobotConfig.LENGTH/2)/MM_PER_INCH,
                        Math.toRadians(-90));
        startingPoses[Alliance.Color.BLUE.ordinal()][StartingPosition.RIGHT.ordinal()] =
                new Pose2d(
                        (-2*Field.TILE_WIDTH + RobotConfig.WIDTH/2)/MM_PER_INCH,
                        (Field.FIELD_WIDTH/2-RobotConfig.LENGTH/2)/MM_PER_INCH,
                        Math.toRadians(-90));
    }
    public static final Pose2d[][] hubDeliveryPoses = new Pose2d[Alliance.Color.values().length][StartingPosition.values().length];
    {
        hubDeliveryPoses[Alliance.Color.RED.ordinal()][StartingPosition.LEFT.ordinal()] =
                new Pose2d(
                        -1.5*Field.TILE_WIDTH/MM_PER_INCH,
                        -Field.TILE_WIDTH/MM_PER_INCH,
                        Math.toRadians(180));
        hubDeliveryPoses[Alliance.Color.RED.ordinal()][StartingPosition.RIGHT.ordinal()] =
                new Pose2d(
                        0,
                        -2*Field.TILE_WIDTH/MM_PER_INCH,
                        Math.toRadians(135));
        hubDeliveryPoses[Alliance.Color.BLUE.ordinal()][StartingPosition.LEFT.ordinal()] =
                new Pose2d(
                        0,
                        2*Field.TILE_WIDTH/MM_PER_INCH,
                        Math.toRadians(-135));
        hubDeliveryPoses[Alliance.Color.BLUE.ordinal()][StartingPosition.RIGHT.ordinal()] =
                new Pose2d(
                        -1.5*Field.TILE_WIDTH/MM_PER_INCH,
                        Field.TILE_WIDTH/MM_PER_INCH,
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
            RobotLog.i("SilverTitans: Field initialization started");
            if (alliance == Alliance.Color.RED) {
                if (startingPosition == StartingPosition.LEFT) {
                    reachCarouselTrajectory =
                            MecanumDriveTrain.trajectoryBuilder(startingPoses[alliance.ordinal()][startingPosition.ordinal()])
                                    .splineToLinearHeading(spinningPoses[alliance.ordinal()], 0).build();
                    reachHubTrajectory =
                            MecanumDriveTrain.trajectoryBuilder(reachCarouselTrajectory.end(), true)
                                    .splineTo(new Vector2d(-54.0, -24.0), Math.toRadians(0.0))
                                    .splineTo(new Vector2d(-36.0, -24.0), 0)
                                    .build();
                    navigateTrajectory = MecanumDriveTrain.trajectoryBuilder(reachHubTrajectory.end())
                            .splineToConstantHeading(new Vector2d(-48.0, -24.0), Math.toRadians(180.0))
                            .splineToConstantHeading(new Vector2d(-60.0, -35.5), Math.toRadians(180.0))
                            .build();
                }
                else {
                    reachHubTrajectory =
                            MecanumDriveTrain.trajectoryBuilder(startingPoses[alliance.ordinal()][startingPosition.ordinal()])
                                    .splineToConstantHeading(new Vector2d(-12.0, -48.0), Math.toRadians(90.0))
                                    .build();
                    navigateTrajectory = MecanumDriveTrain.trajectoryBuilder(reachHubTrajectory.end())
                            .splineTo(new Vector2d(0.0, -49.0), Math.toRadians(0.0))
                            .splineTo(new Vector2d(30.0, -45.75), 0.0)
                            .splineTo(new Vector2d(60.0, -45.75), 0.0)
                            .build();
                }
            }
            if (alliance == Alliance.Color.BLUE) {
                if (startingPosition == StartingPosition.RIGHT) {
                    reachCarouselTrajectory =
                            MecanumDriveTrain.trajectoryBuilder(startingPoses[alliance.ordinal()][startingPosition.ordinal()])
                                    .splineToLinearHeading(spinningPoses[alliance.ordinal()], 0).build();
                    reachHubTrajectory =
                            MecanumDriveTrain.trajectoryBuilder(reachCarouselTrajectory.end(), true)
                                    .splineTo(new Vector2d(-54.0, 24.0), Math.toRadians(0.0))
                                    .splineTo(new Vector2d(-36.0, 24.0), 0)
                                    .build();
                    navigateTrajectory = MecanumDriveTrain.trajectoryBuilder(reachHubTrajectory.end())
                            .splineToConstantHeading(new Vector2d(-48.0, 24.0), Math.toRadians(180.0))
                            .splineToConstantHeading(new Vector2d(-60.0, 35.5), Math.toRadians(180.0))
                            .build();
                }
                else {
                    reachHubTrajectory =
                            MecanumDriveTrain.trajectoryBuilder(startingPoses[alliance.ordinal()][startingPosition.ordinal()])
                                    .splineToConstantHeading(new Vector2d(-12.0, 48.0), Math.toRadians(-90.0))
                                    .build();
                    navigateTrajectory = MecanumDriveTrain.trajectoryBuilder(reachHubTrajectory.end())
                            .splineTo(new Vector2d(0.0, 49.0), Math.toRadians(0.0))
                            .splineTo(new Vector2d(30.0, 45.75), 0.0)
                            .splineTo(new Vector2d(60.0, 45.75), 0.0)
                            .build();
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