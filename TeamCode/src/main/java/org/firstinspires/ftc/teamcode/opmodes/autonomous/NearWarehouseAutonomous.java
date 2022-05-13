package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.robot.RobotConfig;
import org.firstinspires.ftc.teamcode.robot.operations.BearingOperation;
import org.firstinspires.ftc.teamcode.robot.operations.DriveInDirectionOperation;
import org.firstinspires.ftc.teamcode.robot.operations.FollowTrajectory;
import org.firstinspires.ftc.teamcode.robot.operations.IntakeOperation;
import org.firstinspires.ftc.teamcode.robot.operations.OutputOperation;
import org.firstinspires.ftc.teamcode.robot.operations.State;
import org.firstinspires.ftc.teamcode.robot.operations.WaitOperation;

public abstract class NearWarehouseAutonomous extends AutonomousHelper {
    @Override
    public void start() {
        super.start();
        State state;
        //Create a state to get to the alliance shipping hub and deliver box
        state = new State("Deliver first freight");
        state.addPrimaryOperation(new FollowTrajectory(field.getFirstTimeReachHubTrajectory(), robot.getDriveTrain(), "Reach hub"));
        state.addPrimaryOperation(new OutputOperation(robot.getOutPutter(), robot.getIntake(), OutputOperation.Type.Open, "Deliver first freight"));

        state.addSecondaryOperation(new WaitOperation(500, "Wait half a sec before releasing arm"));
        switch (match.getBarcodeLevel()) {
            case 1: {
                state.addSecondaryOperation(new OutputOperation(robot.getOutPutter(), robot.getIntake(), OutputOperation.Type.Level_Low, "Level low"));
                break;
            }
            case 2: {
                state.addSecondaryOperation(new OutputOperation(robot.getOutPutter(), robot.getIntake(), OutputOperation.Type.Level_Middle, "Level middle"));
                break;
            }
            case 3: {
                state.addSecondaryOperation(new OutputOperation(robot.getOutPutter(), robot.getIntake(), OutputOperation.Type.Level_High, "Level high"));
                break;
            }
        }
        states.add(state);

        state = new State("Get to warehouse first time");

        state.addPrimaryOperation(new FollowTrajectory(field.getFirstTimeReachWallTrajectory(), robot.getDriveTrain(), "Reach wall first time"));
        //state.addPrimaryOperation(new FollowTrajectory(field.getFirstTimeNavigateTrajectory(), robot.getDriveTrain(), "Reach warehouse first time"));
        state.addPrimaryOperation(new DriveInDirectionOperation(1* Field.TILE_WIDTH, 0, RobotConfig.REGULAR_SPEED, robot.getDriveTrain(), "Enter warehouse first time"));

        state.addPrimaryOperation(new BearingOperation(15, robot.getDriveTrain(), "Tilt in warehouse"));
        state.addPrimaryOperation(new DriveInDirectionOperation(200, 15, RobotConfig.REGULAR_SPEED, robot.getDriveTrain(), "Move forward a bit"));

        state.addSecondaryOperation(new WaitOperation(250, "Wait quarter sec before retracting arm"));
        state.addSecondaryOperation(new OutputOperation(robot.getOutPutter(), robot.getIntake(), OutputOperation.Type.Level_Intake, "Intake position"));
        state.addTertiaryOperation(new IntakeOperation(robot.getIntake(), IntakeOperation.Type.Intake, "Intake On"));
        states.add(state);

        state = new State("Deliver first freight from warehouse");
        //state.addPrimaryOperation(new FollowTrajectory(field.getReturnToWallTrajectory(), robot.getDriveTrain(), "Reach wall from warehouse"));
        state.addPrimaryOperation(new DriveInDirectionOperation(-200, 15, RobotConfig.REGULAR_SPEED, robot.getDriveTrain(), "Move backwards a bit"));
        state.addPrimaryOperation(new BearingOperation(0, robot.getDriveTrain(), "Realign in warehouse"));
        state.addPrimaryOperation(new DriveInDirectionOperation(-1* Field.TILE_WIDTH, 0, RobotConfig.REGULAR_SPEED, robot.getDriveTrain(), "Exit warehouse first time"));
        state.addPrimaryOperation(new FollowTrajectory(field.getReturnToHubTrajectory(), robot.getDriveTrain(), "Reach hub second time"));
        state.addPrimaryOperation(new OutputOperation(robot.getOutPutter(), robot.getIntake(), OutputOperation.Type.Open, "Deliver"));

        state.addSecondaryOperation(new WaitOperation(1500, "Wait 1.5 sec before raising arm"));
        state.addSecondaryOperation(new OutputOperation(robot.getOutPutter(), robot.getIntake(), OutputOperation.Type.Level_High, "Level High"));
        states.add(state);

        state = new State("Get to warehouse second time");
        state.addPrimaryOperation(new FollowTrajectory(field.getSubsequentTimeReachWallTrajectory(), robot.getDriveTrain(), "Reach wall second time"));
        //state.addPrimaryOperation(new FollowTrajectory(field.getSubsequentNavigateTrajectory(), robot.getDriveTrain(), "Reach warehouse second time"));
        state.addPrimaryOperation(new DriveInDirectionOperation(1.9* Field.TILE_WIDTH, 0, RobotConfig.REGULAR_SPEED, robot.getDriveTrain(), "Enter warehouse second time"));

        state.addSecondaryOperation(new WaitOperation(250, "Wait quarter sec before retracting arm"));
        state.addSecondaryOperation(new OutputOperation(robot.getOutPutter(), robot.getIntake(), OutputOperation.Type.Level_Intake, "Intake position"));
        state.addTertiaryOperation(new IntakeOperation(robot.getIntake(), IntakeOperation.Type.Intake, "Intake On"));
        states.add(state);

        state = new State("Deliver second freight from warehouse");
        state.addPrimaryOperation(new DriveInDirectionOperation(-1.7* Field.TILE_WIDTH, 0, RobotConfig.REGULAR_SPEED, robot.getDriveTrain(), "Exit warehouse second time"));
        //state.addPrimaryOperation(new FollowTrajectory(field.getReturnToWallTrajectory(), robot.getDriveTrain(), "Reach wall from warehouse"));
        state.addPrimaryOperation(new FollowTrajectory(field.getReturnToHubTrajectory(), robot.getDriveTrain(), "Reach hub second time"));
        state.addPrimaryOperation(new OutputOperation(robot.getOutPutter(), robot.getIntake(), OutputOperation.Type.Open, "Deliver"));

        state.addSecondaryOperation(new WaitOperation(1500, "Wait 1.5 sec before raising arm"));
        state.addSecondaryOperation(new OutputOperation(robot.getOutPutter(), robot.getIntake(), OutputOperation.Type.Level_High, "Level High"));
        states.add(state);

        state = new State("Get to warehouse third time");
        state.addPrimaryOperation(new FollowTrajectory(field.getSubsequentTimeReachWallTrajectory(), robot.getDriveTrain(), "Reach wall third time"));
        //state.addPrimaryOperation(new FollowTrajectory(field.getSubsequentNavigateTrajectory(), robot.getDriveTrain(), "Reach warehouse second time"));
        state.addPrimaryOperation(new DriveInDirectionOperation(1.7* Field.TILE_WIDTH, 0, RobotConfig.REGULAR_SPEED, robot.getDriveTrain(), "Enter warehouse third time"));

        state.addSecondaryOperation(new WaitOperation(500, "Wait half a sec before retracting arm"));
        state.addSecondaryOperation(new OutputOperation(robot.getOutPutter(), robot.getIntake(), OutputOperation.Type.Level_Intake, "Intake position"));
        state.addSecondaryOperation(new IntakeOperation(robot.getIntake(), IntakeOperation.Type.Intake, "Intake On"));
        states.add(state);
    }
}
