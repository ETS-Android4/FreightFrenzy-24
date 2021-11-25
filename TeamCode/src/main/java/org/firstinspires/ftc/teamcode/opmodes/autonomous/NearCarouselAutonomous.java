package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import org.firstinspires.ftc.teamcode.game.Alliance;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.robot.operations.CarouselOperation;
import org.firstinspires.ftc.teamcode.robot.operations.DistanceInDirectionOperation;
import org.firstinspires.ftc.teamcode.robot.operations.DriveForTimeOperation;
import org.firstinspires.ftc.teamcode.robot.operations.FollowTrajectory;
import org.firstinspires.ftc.teamcode.robot.operations.OutputOperation;
import org.firstinspires.ftc.teamcode.robot.operations.State;

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
        states.add(state);

        //Create a state to get to the alliance shipping hub and deliver box
        state = new State("Deliver box");
        state.addPrimaryOperation(new FollowTrajectory(
                Field.reachHubTrajectory,
                robot.getDriveTrain(),
                "Reach hub"));
        double distanceToHub = 0;
        switch (match.getBarcodeLevel()) {
            case 1:
            {
                state.addPrimaryOperation(new OutputOperation(robot.getOutPutter(), OutputOperation.Type.Level_Low, "Level low"));
                break;
            }
            case 2:
            {
                state.addPrimaryOperation(new OutputOperation(robot.getOutPutter(), OutputOperation.Type.Level_Middle, "Level middle"));
                distanceToHub = -6.4*Field.MM_PER_INCH;
                break;
            }
            case 3:
            {
                state.addPrimaryOperation(new OutputOperation(robot.getOutPutter(), OutputOperation.Type.Level_High, "Level high"));
                distanceToHub = -9.5*Field.MM_PER_INCH;
                break;
            }
        }
        state.addPrimaryOperation(new DistanceInDirectionOperation(
                distanceToHub,
                Math.toDegrees(Field.reachHubTrajectory.end().getHeading()),
                .3,
                robot.getDriveTrain(),
                "Approach hub"
        ));

        //TODO - add appropriate forward movement for the delivery level
        state.addPrimaryOperation(new OutputOperation(robot.getOutPutter(), OutputOperation.Type.Deliver, "Place on hub"));
        state.addPrimaryOperation(new OutputOperation(robot.getOutPutter(), OutputOperation.Type.Retract, "Retract"));
        state.addPrimaryOperation(new DistanceInDirectionOperation(
                -distanceToHub,
                Math.toDegrees(Field.reachHubTrajectory.end().getHeading()),
                .3,
                robot.getDriveTrain(),
                "Approach hub"
        ));
        states.add(state);

        //Create a state to get to the alliance storage unit
        state = new State("NavigateToAllianceStorageUnit");
        state.addPrimaryOperation(new OutputOperation(robot.getOutPutter(), OutputOperation.Type.Fold, "Fold"));
        state.addPrimaryOperation(new FollowTrajectory(
                Field.navigateTrajectory,
                robot.getDriveTrain(),"Navigate"));
        states.add(state);
    }
    /*
    Low: -42.5, -24.0
    Middle : -36.1, -24.0
    Top: -32.0, -24

     */
}
