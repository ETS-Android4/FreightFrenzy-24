package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.game.Alliance;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.RobotConfig;
import org.firstinspires.ftc.teamcode.robot.operations.State;

import java.util.ArrayList;
import java.util.Date;
import java.util.Locale;

public abstract class AutonomousHelper extends OpMode {
    protected Match match;
    protected Robot robot;
    protected Field field;

    ArrayList<State> states = new ArrayList<>();

    protected boolean initialOperationsDone;

    Date initStartTime;

    boolean cameraPoseSet = false;
    boolean aPressed;
    boolean statesAdded = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    public void init(Alliance.Color alliance, Field.StartingPosition startingPosition) {
        initStartTime = new Date();
        cameraPoseSet = false;
        aPressed = false;
        statesAdded = false;

        AutoTransitioner.transitionOnStop(this, "Phoebe: Driver Controlled");

        this.match = Match.getNewInstance();
        match.init();
        Match.log("Match initialized, setting alliance to " + alliance
                + " and starting position to " + startingPosition);
        match.setAlliance(alliance);
        match.setStartingPosition(startingPosition);
        field = match.getField();
        field.init(alliance, startingPosition);

        this.robot = match.getRobot();
        Match.log("Initializing robot");
        this.robot.init(hardwareMap, telemetry, match);
        initialOperationsDone = false;
        Match.log("Robot initialized");
        telemetry.update();
    }
    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        if (Field.isNotInitialized()) {
            //RobotLog.i("SilverTitans: Initializing trajectories, please wait");
            telemetry.addData("Status", "Trajectories initializing, please wait. " +
                    (30 - (int)(new Date().getTime() - initStartTime.getTime())/1000));
            telemetry.addData("VSLAM", robot.getVSLAMStatus());
        }
        else if (robot.fullyInitialized()) {
            match.setBarcodeLevel(robot.getBarCodeLevel());
            if (!robot.havePosition()) {
                robot.startVSLAM();
                telemetry.addData("Status", "VSLAM initializing, please wait.");
                telemetry.addData("VSLAM", robot.getVSLAMStatus());
                telemetry.addData("Barcode", match.getBarcodeLevel());
            }
            else if (!cameraPoseSet) {
                telemetry.addData("Status", "Setting position, please wait");
                telemetry.addData("VSLAM", robot.getVSLAMStatus());
                telemetry.addData("Barcode", match.getBarcodeLevel());
                robot.setInitialPose(field.getStartingPose());
                cameraPoseSet = true;
            }
            else {
                double xError = robot.getCurrentX() / Field.MM_PER_INCH - field.getStartingPose().getX();
                double yError = robot.getCurrentY() / Field.MM_PER_INCH - field.getStartingPose().getY();
                double bearingError = (Math.toDegrees(robot.getCurrentTheta())
                        - Math.toDegrees(field.getStartingPose().getHeading())) % 360;
                if ((Math.abs(xError) > RobotConfig.ALLOWED_POSITIONAL_ERROR)
                        || (Math.abs(yError) > RobotConfig.ALLOWED_POSITIONAL_ERROR
                        || (Math.abs(bearingError) > RobotConfig.ALLOWED_BEARING_ERROR))) {
                    String positionError = String.format(Locale.getDefault(),
                            "Position Error, restart app:%s v/s %s, xErr:%.2f, yErr:%.2f, hErr:%.2fvs%.2f=%.2f",
                            field.getStartingPose(),
                            robot.getPosition(),
                            xError,
                            yError,
                            Math.toDegrees(robot.getCurrentTheta()),
                            Math.toDegrees(field.getStartingPose().getHeading()),
                            bearingError);
                    telemetry.addData("Status", positionError);
                    telemetry.addData("VSLAM", robot.getVSLAMStatus());
                    telemetry.addData("Barcode", match.getBarcodeLevel());
                } else {
                    match.updateTelemetry(telemetry, "Ready, let's go");
                }
            }
        }
        else {
            telemetry.addData("Status", "Cameras initializing, please wait");
        }
        telemetry.update();
    }

    @Override
    public void start() {
        match.setStart();
    }

    /**
     * We go through our specified desired states in this method.
     * Loop through the states, checking if a state is reached, if it is not reached, queue
     *         it if not already queued
     */
    @Override
    public void loop() {
        /*
        Check states sequentially. Skip over reached states and queue those that have not
        been reached and not yet queued
         */
        for (State state : states) {
            if (!state.isReached(robot)) {
                if (state.isQueued()) {
                    match.updateTelemetry(telemetry, "Attempting " + state.getTitle());
                } else {
                    //queue state if it has not been queued
                    match.updateTelemetry(telemetry, "Queueing " + state.getTitle());
                    Match.log("Queueing state: " + state.getTitle());
                    state.queue(robot);
                }
                break;
            }
        }
    }

    @Override
    public void stop() {
        this.robot.stop();
    }
}