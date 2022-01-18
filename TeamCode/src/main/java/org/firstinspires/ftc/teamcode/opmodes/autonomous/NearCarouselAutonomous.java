package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import org.firstinspires.ftc.teamcode.game.Alliance;
import org.firstinspires.ftc.teamcode.robot.operations.CarouselOperation;
import org.firstinspires.ftc.teamcode.robot.operations.DriveForTimeOperation;
import org.firstinspires.ftc.teamcode.robot.operations.FollowTrajectory;
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
                field.getReachCarouselTrajectory(),
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
        state.addSecondaryOperation(new WaitOperation(500, "Wait half a sec before releasing shoulder"));
        switch (match.getBarcodeLevel()) {
            case 1: {
                state.addSecondaryOperation(new OutputOperation(robot.getOutPutter(), robot.getIntake(), OutputOperation.Type.Level_Low, "Low"));
                break;
            }
            case 2: {
                state.addSecondaryOperation(new OutputOperation(robot.getOutPutter(), robot.getIntake(), OutputOperation.Type.Level_Middle, "Middle"));
                break;
            }
            case 3: {
                state.addSecondaryOperation(new OutputOperation(robot.getOutPutter(), robot.getIntake(), OutputOperation.Type.Level_High, "High"));
                break;
            }
        }
        state.setCompletionBasedUpon(State.CompletionBasedUpon.PRIMARY_OPERATIONS);
        states.add(state);

        //Create a state to get to the alliance shipping hub and deliver box
        state = new State("Deliver box");
        state.addPrimaryOperation(new FollowTrajectory(
                field.getFirstTimeReachHubTrajectory(),
                robot.getDriveTrain(),
                "Reach hub"));
        state.addPrimaryOperation(new OutputOperation(robot.getOutPutter(), robot.getIntake(), OutputOperation.Type.Open, "Place on hub"));
        state.addPrimaryOperation(new WaitOperation(500, "Wait before closing lid"));
        state.addPrimaryOperation(new OutputOperation(robot.getOutPutter(), robot.getIntake(), OutputOperation.Type.Close, "Close lid"));
        states.add(state);

        //Create a state to get to the alliance shipping area
        state = new State("Navigate");
        state.setCompletionBasedUpon(State.CompletionBasedUpon.ALL_OPERATIONS);
        state.addPrimaryOperation(new FollowTrajectory(
                field.getFirstTimeNavigateTrajectory(),
                robot.getDriveTrain(),"Navigate"));
        state.addSecondaryOperation(new OutputOperation(robot.getOutPutter(), robot.getIntake(), OutputOperation.Type.Vertical, "Vertical"));
        states.add(state);
    }
}
