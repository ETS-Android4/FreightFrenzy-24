package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import org.firstinspires.ftc.teamcode.game.Alliance;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.robot.RobotConfig;
import org.firstinspires.ftc.teamcode.robot.operations.DriveForTimeOperation;
import org.firstinspires.ftc.teamcode.robot.operations.FollowTrajectory;
import org.firstinspires.ftc.teamcode.robot.operations.OutputOperation;
import org.firstinspires.ftc.teamcode.robot.operations.State;
import org.firstinspires.ftc.teamcode.robot.operations.StrafeLeftForDistanceOperation;
public abstract class NearWarehouseAutonomous extends AutonomousHelper {
    @Override
    public void start() {
        super.start();
        State state;
        //Create a state to get to the alliance shipping hub and deliver box
        state = new State("Deliver box");
        state.addPrimaryOperation(new FollowTrajectory(
                Field.reachHubTrajectory,
                robot.getDriveTrain(),"Reach hub"));
        switch (match.getBarcodeLevel()) {
            case 1:
            {
                state.addSecondaryOperation(new OutputOperation(robot.getOutPutter(), robot.getIntake(), OutputOperation.Type.Level_Low, "Level low"));
                break;
            }
            case 2:
            {
                state.addSecondaryOperation(new OutputOperation(robot.getOutPutter(), robot.getIntake(), OutputOperation.Type.Level_Middle, "Level middle"));
                break;
            }
            case 3:
            {
                state.addSecondaryOperation(new OutputOperation(robot.getOutPutter(), robot.getIntake(), OutputOperation.Type.Level_High, "Level high"));
                break;
            }
        }
        //TODO - add appropriate forward movement required for the delivery level
        //state.addPrimaryOperation(new OutputOperation(robot.getOutPutter(), OutputOperation.Type.Deliver, "Place on hub"));
        states.add(state);

        //Create a state to get to the warehouse
        state = new State("NavigateToWarehouse");
        state.addPrimaryOperation(new FollowTrajectory(
                Field.navigateTrajectory,
                robot.getDriveTrain(),"Navigate"));
        state.addSecondaryOperation(new OutputOperation(robot.getOutPutter(), robot.getIntake(), OutputOperation.Type.Level_Intake, "Intake position"));
        //Strafe left to push scoring element if alliance is red, strafe right otherwise
        if (match.getAlliance() == Alliance.Color.RED) {
            state.addPrimaryOperation(new StrafeLeftForDistanceOperation(
                    6 * Field.MM_PER_INCH,
                    RobotConfig.REGULAR_SPEED,
                    robot.getDriveTrain(),
                    "Strafe left to push element"));
        }
        else {
            state.addPrimaryOperation(new StrafeLeftForDistanceOperation(
                    -6 * Field.MM_PER_INCH,
                    RobotConfig.REGULAR_SPEED,
                    robot.getDriveTrain(),
                    "Strafe right to push element"));
        }
        state.addPrimaryOperation(new DriveForTimeOperation(3000, 0, RobotConfig.REGULAR_SPEED, robot.getDriveTrain(), "Reach warehouse"));
        states.add(state);
    }
}
