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

    protected boolean wobbleLifted, queuedWobbleLift;
    protected boolean numberOfRingsDetermined, determinationQueued;
    protected boolean firstWobbleDeposited, firstWobbleDepositQueued;
    protected boolean navigated, navigationQueued;
    protected boolean ringsShootingPositionReached, ringShootingPositionQueued;
    protected boolean firstRingShot, firstRingShootingQueued;
    protected boolean secondRingShot, secondRingShootingQueued;
    protected boolean thirdRingShot, thirdRingShootingQueued;
    protected boolean returnedForSecondWobble, returnForSecondWobbleQueued;
    protected boolean collectedSecondWobble, collectionOfSecondWobbleQueued;
    protected boolean secondWobbleGoalDeposited, secondWobbleGoalDepositQueued;
    protected boolean wobbleRaisedInitially;
    boolean cameraWorks = false;

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
        field.init();

        this.robot = match.getRobot();
        Match.log("Initializing robot");
        this.robot.init(hardwareMap, telemetry, match);
        wobbleRaisedInitially = false;
        cameraWorks = false;
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
            if (!wobbleRaisedInitially) {
                robot.setPose(Field.STARTING_POSE);
                //robot.startVSLAM();
                Match.log("Init complete");
            }
            else if (!robot.havePosition()) {
                telemetry.addData("Status", "Waiting for position, please wait");
            }
            else if (robot.allOperationsCompleted()) {
                    double xError = robot.getCurrentX() / Field.MM_PER_INCH - Field.STARTING_POSE.getX();
                    double yError = robot.getCurrentY() / Field.MM_PER_INCH - Field.STARTING_POSE.getY();
                    double bearingError = AngleUnit.normalizeRadians(robot.getCurrentTheta()) - AngleUnit.normalizeRadians(Field.STARTING_POSE.getHeading());
                    if ((Math.abs(xError) > RobotConfig.ALLOWED_POSITIONAL_ERROR)
                            || (Math.abs(yError) > RobotConfig.ALLOWED_POSITIONAL_ERROR
                            || (Math.abs(bearingError) > RobotConfig.ALLOWED_BEARING_ERROR))) {
                        telemetry.addData("Status", "PositionError:" + Field.STARTING_POSE + " v/s" + robot.getPosition() + ", please init again");
                        robot.setPose(Field.STARTING_POSE);
                    } else {
                        match.updateTelemetry(telemetry,"Ready, let's go");
                    }
            }
            else {
                telemetry.addData("Status", "Waiting for wobble to lift, please wait");
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