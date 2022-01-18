package org.firstinspires.ftc.teamcode.game;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.robot.RobotConfig;

import static org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive.accurateTrajectoryBuilder;

/**
 * Created by Silver Titans on 9/16/17.
 */

/*
       X: -.5
       y: -51
       theta: 61.7
 */
public class Field {
    public static final float MM_PER_INCH = 25.4f;
    public static final double M_PER_INCH = MM_PER_INCH/1000;
    //the width of each tile
    public static final double TILE_WIDTH = 24 * MM_PER_INCH;
    // the width of the FTC field (from the center point to the outer panels)
    public static final double FIELD_WIDTH = 6 * TILE_WIDTH;

    //the center point of robot travel to get through the channel
    public static final double CHANNEL_CENTER = 69;

    public static final double FORWARD_DISTANCE_FOR_BARCODE_1 = 17.0;
    public static final double FORWARD_DISTANCE_FOR_BARCODE_2 = 15.0;
    public static final double FORWARD_DISTANCE_FOR_BARCODE_3 = 19.0;

    public static final double HUB_APPROACH_ANGLE = -150;

    public static volatile boolean initialized = false;
    public static final Object mutex = new Object();

    public enum StartingPosition {
        Left, Right
    }

    public static final Pose2d redLeftStartingPose = new Pose2d(
                        (-2*Field.TILE_WIDTH + RobotConfig.ROBOT_WIDTH/2)/MM_PER_INCH,
                        (-Field.FIELD_WIDTH/2 + RobotConfig.ROBOT_CENTER_FROM_BACK)/MM_PER_INCH,
                        Math.toRadians(90));
    public static final Pose2d redRightStartingPose =
                new Pose2d(
                        (RobotConfig.ROBOT_WIDTH/2)/MM_PER_INCH,
                        (-Field.FIELD_WIDTH/2 + RobotConfig.ROBOT_CENTER_FROM_BACK)/MM_PER_INCH,
                        Math.toRadians(90));
    public static final Pose2d blueLeftStartingPose =
                new Pose2d(
                        (RobotConfig.ROBOT_WIDTH/2)/MM_PER_INCH,
                        (Field.FIELD_WIDTH/2 - RobotConfig.ROBOT_CENTER_FROM_BACK)/MM_PER_INCH,
                        Math.toRadians(-90));
    public static final Pose2d blueRightStartingPose =
                new Pose2d(
                        (-2*Field.TILE_WIDTH + RobotConfig.ROBOT_WIDTH/2)/MM_PER_INCH,
                        (Field.FIELD_WIDTH /2 - RobotConfig.ROBOT_CENTER_FROM_BACK)/MM_PER_INCH,
                        Math.toRadians(-90));
    public static final Pose2d redSpinningPose =
                new Pose2d(
                        -58,
                        -58,
                        Math.toRadians(225));
    public static final Pose2d blueSpinningPose =
                new Pose2d(
                        -58,
                        58,
                        Math.toRadians(135));

    public Trajectory getReachCarouselTrajectory() {
        return reachCarouselTrajectory;
    }

    Trajectory reachCarouselTrajectory = null;

    public Trajectory getFirstTimeReachHubTrajectory() {
        return firstTimeReachHubTrajectories[Match.getInstance().getBarcodeLevel()-1];
    }

    private final Trajectory[] firstTimeReachHubTrajectories = new Trajectory[3];

    public Trajectory getFirstTimeReachWallTrajectory() {
        return firstTimeReachWallTrajectories[Match.getInstance().getBarcodeLevel()-1];
    }

    public Trajectory getSubsequentTimeReachWallTrajectory() {
        return firstTimeReachWallTrajectories[2];
    }

    private Trajectory[] firstTimeReachWallTrajectories = new Trajectory[3];

    public Trajectory getFirstTimeNavigateTrajectory() {
        return firstTimeNavigateTrajectories[Match.getInstance().getBarcodeLevel()-1];
    }
    public Trajectory getSubsequentNavigateTrajectory() {
        return firstTimeNavigateTrajectories[2];
    }
    private Trajectory[] firstTimeNavigateTrajectories = new Trajectory[3];

    public Trajectory getReturnToWallTrajectory() {
        return returnToWallTrajectory;
    }

    private Trajectory returnToWallTrajectory;

    public Trajectory getReturnToHubTrajectory() {
        return returnToHubTrajectory;
    }

    private Trajectory returnToHubTrajectory;

    private Pose2d startingPose;

    public void init(Alliance.Color alliance, StartingPosition startingPosition) {
        switch (alliance) {
            case RED: {
                switch (startingPosition) {
                    case Left: {
                        startingPose = redLeftStartingPose;
                        break;
                    }
                    case Right: {
                        startingPose = redRightStartingPose;
                        break;
                    }
                }
                break;
            }
            case BLUE: {
                switch (startingPosition) {
                    case Left: {
                        startingPose = blueLeftStartingPose;
                        break;
                    }
                    case Right: {
                        startingPose = blueRightStartingPose;
                        break;
                    }
                }
                break;
            }
        }
        boolean nearCarousel = startingPose.getX() < 0;
        Thread initThread = new Thread(() -> {
            Match.log("Field initialization started for "
                    + alliance.toString() + ", " + startingPosition.toString() + ", near " + (nearCarousel ? "carousel" : "warehouse"));
            double yMultiplier;
            if (alliance == Alliance.Color.RED) {
                yMultiplier = -1;
            }
            else {
                yMultiplier = 1;
            }
            if (nearCarousel) {
                createNearCarouselTrajectories(alliance, yMultiplier);
            } else { // Near warehouse
                createNearWarehouseTrajectories(yMultiplier);
            }

            synchronized (mutex) {
                initialized = true;
            }
        });
        initThread.start();
    }

    private void createNearWarehouseTrajectories(double yMultiplier) {
        //create the reach hub trajectories for three different bar code levels
        firstTimeReachHubTrajectories[0] =
                accurateTrajectoryBuilder(getStartingPose())
                    .splineToLinearHeading(getStartingPose().minus(new Pose2d(4, yMultiplier *FORWARD_DISTANCE_FOR_BARCODE_1, Math.toRadians(yMultiplier *HUB_APPROACH_ANGLE))),
                            0)
                    .build();
        Thread.yield();
        Match.log("Created reach hub trajectory for bottom level");

        firstTimeReachHubTrajectories[1] =
                accurateTrajectoryBuilder(getStartingPose())
                        .splineToLinearHeading(getStartingPose().minus(new Pose2d(4, yMultiplier *FORWARD_DISTANCE_FOR_BARCODE_2, Math.toRadians(yMultiplier *HUB_APPROACH_ANGLE))),
                                0)
                        .build();
        Thread.yield();
        Match.log("Created reach hub trajectory for mid level");

        firstTimeReachHubTrajectories[2] =
                accurateTrajectoryBuilder(getStartingPose())
                        .splineToLinearHeading(getStartingPose().minus(new Pose2d(4, yMultiplier *FORWARD_DISTANCE_FOR_BARCODE_3, Math.toRadians(yMultiplier *HUB_APPROACH_ANGLE))),
                                0)
                .build();
        Thread.yield();
        Match.log("Created reach hub trajectory for top level");

        //trajectories to get parallel to wall near starting position
        for (int i=0; i < 3; i++) {
            firstTimeReachWallTrajectories[i] = accurateTrajectoryBuilder(firstTimeReachHubTrajectories[i].end())
                    .splineToLinearHeading(new Pose2d(getStartingPose().getX(), CHANNEL_CENTER * yMultiplier, 0.0), 0.0)
                    .build();
            Thread.yield();
        }
        Match.log("Built trajectories to reach wall");

        //trajectories to navigate
        for (int i=0; i < 3; i++) {
            firstTimeNavigateTrajectories[i] = accurateTrajectoryBuilder(firstTimeReachWallTrajectories[i].end())
                    .splineTo(new Vector2d(40.0, CHANNEL_CENTER* yMultiplier), 0.0)
                    .build();
            Thread.yield();
        }
        Match.log("Built trajectories to navigate");

        returnToWallTrajectory = accurateTrajectoryBuilder(firstTimeNavigateTrajectories[0].end(), true)
            .splineToLinearHeading(new Pose2d(startingPose.getX(), CHANNEL_CENTER* yMultiplier, 0.0), 0)
            .build();
        Thread.yield();
        Match.log("Created return to wall trajectory");

        returnToHubTrajectory = accurateTrajectoryBuilder(returnToWallTrajectory.end(), true)
                .splineToLinearHeading(startingPose.minus(new Pose2d(4, yMultiplier *FORWARD_DISTANCE_FOR_BARCODE_3, Math.toRadians(yMultiplier *HUB_APPROACH_ANGLE))), 0.0)
                .build();
        Thread.yield();
        Match.log("Created return to hub trajectory");
    }

    private void createNearCarouselTrajectories(Alliance.Color alliance, double yMultiplier) {
        switch (alliance) {
            case RED: {
                reachCarouselTrajectory =
                        accurateTrajectoryBuilder(startingPose)
                                .splineTo(redSpinningPose.vec(), Math.toRadians(yMultiplier *135)).build();
                break;
            }
            case BLUE: {
                reachCarouselTrajectory =
                        accurateTrajectoryBuilder(startingPose)
                                .splineTo(blueSpinningPose.vec(), Math.toRadians(yMultiplier *135)).build();
                break;
            }
        }
        Thread.yield();
        Match.log("Created reach carousel trajectory");

        firstTimeReachHubTrajectories[0] =
                accurateTrajectoryBuilder(reachCarouselTrajectory.end(), true)
                .splineTo(new Vector2d(-50.0, 24.0* yMultiplier), 0.0)
                .splineTo(new Vector2d(-42.2, 24.0* yMultiplier), 0.0)
                        .build();
        Thread.yield();
        Match.log("Created reach hub trajectory for bottom level");
        firstTimeReachHubTrajectories[1] =
                accurateTrajectoryBuilder(reachCarouselTrajectory.end(), true)
                        .splineTo(new Vector2d(-50.0, 24.0* yMultiplier), 0.0)
                        .splineTo(new Vector2d(-42.6, 24.0* yMultiplier), 0.0)
                        .build();
        Thread.yield();
        Match.log("Created reach hub trajectory for mid level");
        firstTimeReachHubTrajectories[2] =
                accurateTrajectoryBuilder(reachCarouselTrajectory.end(), true)
                        .splineTo(new Vector2d(-50.0, 24.0* yMultiplier), 0.0)
                        .splineTo(new Vector2d(-40.0, 24.0* yMultiplier), 0.0)
                        .build();
        Thread.yield();
        Match.log("Created reach hub trajectory for top level");

        firstTimeNavigateTrajectories[0] =
                accurateTrajectoryBuilder(firstTimeReachHubTrajectories[0].end())
                    .splineToLinearHeading(new Pose2d(-60, yMultiplier *36 , -45* yMultiplier), 0)
                .build();
        Thread.yield();
        Match.log("Created navigate trajectory for bottom level");

        firstTimeNavigateTrajectories[1] =
                accurateTrajectoryBuilder(firstTimeReachHubTrajectories[1].end())
                        .splineToLinearHeading(new Pose2d(-60, yMultiplier *36 , -45* yMultiplier), 0)
                        .build();
        Thread.yield();
        Match.log("Created navigate trajectory for bottom level");

        firstTimeNavigateTrajectories[2] =
                accurateTrajectoryBuilder(firstTimeReachHubTrajectories[2].end())
                        .splineToLinearHeading(new Pose2d(-60, yMultiplier *36 , -45* yMultiplier), 0)
                        .build();
        Thread.yield();
        Match.log("Created navigate trajectory for top level");
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