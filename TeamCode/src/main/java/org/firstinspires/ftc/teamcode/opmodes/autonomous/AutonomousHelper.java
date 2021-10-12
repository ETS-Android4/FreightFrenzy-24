package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.game.Alliance;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.RobotConfig;
import org.firstinspires.ftc.teamcode.robot.operations.State;

import java.util.ArrayList;
import java.util.Date;

public abstract class AutonomousHelper extends OpMode {
    protected Match match;
    protected Robot robot;
    protected Field field;

    ArrayList<State> states = new ArrayList<>();

    protected boolean initialOperationsDone;

    Date initStartTime;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    public void init(Alliance.Color alliance, Field.StartingPosition startingPosition) {

        initStartTime = new Date();

        AutoTransitioner.transitionOnStop(this, "Phoebe: Driver Controlled");

        this.match = Match.getNewInstance();
        Match.log("Created new match, initializing it");
        match.init();
        Match.log("Match initialized, setting alliance to " + Alliance.Color.RED + " and starting position to " + Field.StartingPosition.RIGHT);
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
        if (!Field.isInitialized()) {
            //RobotLog.i("SilverTitans: Initializing trajectories, please wait");
            telemetry.addData("Status", "Trajectories initializing, please wait. " +
                    (30 - (int)(new Date().getTime() - initStartTime.getTime())/1000));
        }
        else if (robot.fullyInitialized()) {
            if (!initialOperationsDone) {
                initialOperationsDone = true;
                robot.setPose(field.getStartingPose());
            }
            else if (!robot.havePosition()) {
                telemetry.addData("Status", "Waiting for position, please wait");
            }
            else if (robot.allOperationsCompleted()) {
                    double xError = robot.getCurrentX() / Field.MM_PER_INCH - field.getStartingPose().getX();
                    double yError = robot.getCurrentY() / Field.MM_PER_INCH - field.getStartingPose().getY();
                    double bearingError = AngleUnit.normalizeRadians(robot.getCurrentTheta()) - AngleUnit.normalizeRadians(field.getStartingPose().getHeading());
                    if ((Math.abs(xError) > RobotConfig.ALLOWED_POSITIONAL_ERROR)
                            || (Math.abs(yError) > RobotConfig.ALLOWED_POSITIONAL_ERROR
                            || (Math.abs(bearingError) > RobotConfig.ALLOWED_BEARING_ERROR))) {
                        telemetry.addData("Status", "PositionError:" + field.getStartingPose() + " v/s" + robot.getPosition() + ", please init again");
                        robot.setPose(field.getStartingPose());
                    } else {
                        match.updateTelemetry(telemetry,"Ready, let's go");
                    }
            }
            else {
                telemetry.addData("Status", "Waiting for initial operations, please wait");
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

    @Override
    public void loop() {
        for (State state: states) {
            if (!state.isReached(robot)) {
                if (!state.isQueued()) {
                    state.queue(robot);
                }
            }
        }
        match.updateTelemetry(telemetry, "Autonomous");
    }

    protected void queueRingDetermination() {
        //generate the rest of the trajectories - hopefully this completes before we are done depositing first wobble
        match.getField().generateTrajectories();
    }

    @Override
    public void stop() {
        this.robot.stop();
    }
}