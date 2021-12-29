package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import org.firstinspires.ftc.teamcode.game.Alliance;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.robot.RobotConfig;
import org.firstinspires.ftc.teamcode.robot.operations.CarouselOperation;
import org.firstinspires.ftc.teamcode.robot.operations.DistanceInDirectionOperation;
import org.firstinspires.ftc.teamcode.robot.operations.DriveForTimeOperation;
import org.firstinspires.ftc.teamcode.robot.operations.FollowTrajectory;
import org.firstinspires.ftc.teamcode.robot.operations.IntakeOperation;
import org.firstinspires.ftc.teamcode.robot.operations.OutputOperation;
import org.firstinspires.ftc.teamcode.robot.operations.State;
import org.firstinspires.ftc.teamcode.robot.operations.WaitOperation;

public abstract class NearCarouselAutonomous extends AutonomousHelper {

    @Override
    public void start() {
        super.start();
        State state;
        //Create a state to deliver the duck
        state = new State("Deliver Duck");
        state.addPrimaryOperation(new FollowTrajectory(
                Field.reachCarouselTrajectory,
                robot.getDriveTrain(),
                "Reach carousel"));
        state.addPrimaryOperation(
                new DriveForTimeOperation(
                        500, 0,
                        0.3,
                        robot.getDriveTrain(),
                        "Approach Carousel"));
        state.addPrimaryOperation(
                new CarouselOperation(
                        robot.getCarouselSpinner(),
                        match.getAlliance() != Alliance.Color.RED,
                        "Deliver duck"));
        state.addSecondaryOperation(new WaitOperation(1000, "Wait a sec before releasing shoulder"));
        state.addSecondaryOperation(new OutputOperation(robot.getOutPutter(), robot.getIntake(), OutputOperation.Type.Level_Initial_Release, "Initial Release"));
        double distanceToReachHub = 0;
        switch (match.getBarcodeLevel()) {
            case 1: {
                state.addSecondaryOperation(new OutputOperation(robot.getOutPutter(), robot.getIntake(), OutputOperation.Type.Level_Low, "Low"));
                distanceToReachHub = Field.MIDDLE_Y_POSITION - Field.BOTTOM_Y_POSITION;
                break;
            }
            case 2: {
                state.addSecondaryOperation(new OutputOperation(robot.getOutPutter(), robot.getIntake(), OutputOperation.Type.Level_Middle, "Middle"));
                break;
            }
            case 3: {
                state.addSecondaryOperation(new OutputOperation(robot.getOutPutter(), robot.getIntake(), OutputOperation.Type.Level_High, "High"));
                distanceToReachHub = Field.MIDDLE_Y_POSITION - Field.TOP_Y_POSITION;;
                break;
            }
        }
        states.add(state);

        //Create a state to get to the alliance shipping hub and deliver box
        state = new State("Deliver box");
        state.addPrimaryOperation(new FollowTrajectory(
                Field.reachHubTrajectory,
                robot.getDriveTrain(),
                "Reach hub"));
        state.addPrimaryOperation(
                new DistanceInDirectionOperation(
                        -distanceToReachHub,
                        match.getAlliance() == Alliance.Color.RED ? 90 : -90,
                        RobotConfig.SUPER_CAUTIOUS_SPEED,
                        robot.getDriveTrain(),
                        "Correct distance"));
        state.addPrimaryOperation(new OutputOperation(robot.getOutPutter(), robot.getIntake(), OutputOperation.Type.Open, "Place on hub"));
        states.add(state);

        //Create a state to get to the alliance warehouse to grab
        state = new State("NavigateToIntake");
        state.setCompletionBasedUpon(State.CompletionBasedUpon.ALL_OPERATIONS);
        state.addPrimaryOperation(new FollowTrajectory(
                Field.navigateTrajectory,
                robot.getDriveTrain(),"Navigate To Grab"));
        state.addSecondaryOperation(new WaitOperation(500, "Wait before intake position"));
        state.addSecondaryOperation(new OutputOperation(robot.getOutPutter(), robot.getIntake(), OutputOperation.Type.Level_Intake, "Intake position"));
        state.addSecondaryOperation(new IntakeOperation(robot.getIntake(), IntakeOperation.Type.Intake, "Intake On"));
        states.add(state);

        //Create a state to get to intake
        state = new State("Intake");
        state.addPrimaryOperation(new IntakeOperation(robot.getIntake(), IntakeOperation.Type.Consume, "Consume"));
        state.addPrimaryOperation(new WaitOperation(300, "Wait before raising arm"));
        state.addPrimaryOperation(new OutputOperation(robot.getOutPutter(), robot.getIntake(), OutputOperation.Type.Level_High, "High"));
        state.addPrimaryOperation(new FollowTrajectory(
                Field.reachHubAgainTrajectory,
                robot.getDriveTrain(),"Reach hub again"));
        state.addPrimaryOperation(new OutputOperation(robot.getOutPutter(), robot.getIntake(), OutputOperation.Type.Open, "Place"));
        states.add(state);

        //Create a state to get to the alliance warehouse
        state = new State("NavigateToWarehouseAgain");
        state.addSecondaryOperation(new WaitOperation(500, "Wait before intake position"));
        state.addSecondaryOperation(new OutputOperation(robot.getOutPutter(), robot.getIntake(), OutputOperation.Type.Level_Intake, "Intake position"));
        state.addPrimaryOperation(new FollowTrajectory(
                Field.navigateTrajectory,
                robot.getDriveTrain(),"Navigate"));
        states.add(state);
    }
}
