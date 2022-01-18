package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import org.firstinspires.ftc.teamcode.game.Alliance;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.RobotConfig;
import org.firstinspires.ftc.teamcode.robot.operations.BearingOperation;
import org.firstinspires.ftc.teamcode.robot.operations.DriveForDynamicDistanceOperation;
import org.firstinspires.ftc.teamcode.robot.operations.DriveInDirectionOperation;
import org.firstinspires.ftc.teamcode.robot.operations.DriveUntilFreightOperation;
import org.firstinspires.ftc.teamcode.robot.operations.FollowTrajectory;
import org.firstinspires.ftc.teamcode.robot.operations.IntakeOperation;
import org.firstinspires.ftc.teamcode.robot.operations.OutputOperation;
import org.firstinspires.ftc.teamcode.robot.operations.State;
import org.firstinspires.ftc.teamcode.robot.operations.WaitOperation;

public abstract class NearWarehouseAutonomousV2 extends AutonomousHelper {
    public static final double FIRST_ENTRY_DISTANCE = 1.5*Field.TILE_WIDTH;
    public static final double SECOND_ENTRY_DISTANCE = 1.5*Field.TILE_WIDTH;
    @Override
    public void start() {
        super.start();

        deliverFirstFreight();
        enterWarehouseFirstTime();
        deliverSecondFreight();
        enterWarehouseSecondTime();
        deliverThirdFreight();
        enterWarehouseThirdTime();
    }

    private void deliverFirstFreight() {
        State state;
        //Create a state to get to the alliance shipping hub and deliver box
        state = new State("Deliver box");

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
    }
    private void enterWarehouseFirstTime() {
        State state;
        state = new State("Get to warehouse first time");

        state.addPrimaryOperation(new FollowTrajectory(field.getFirstTimeReachWallTrajectory(), robot.getDriveTrain(), "Reach wall first time"));
        //state.addPrimaryOperation(new FollowTrajectory(field.getFirstTimeNavigateTrajectory(), robot.getDriveTrain(), "Reach warehouse first time"));
        state.addPrimaryOperation(new DriveInDirectionOperation(FIRST_ENTRY_DISTANCE, 0, RobotConfig.REGULAR_SPEED, robot.getDriveTrain(), "Enter warehouse first time"));
        state.addPrimaryOperation(new BearingOperation(Match.getInstance().getAlliance() == Alliance.Color.RED ? 30 : -30,  robot.getDriveTrain(), "Point away"));
        state.addPrimaryOperation(new DriveUntilFreightOperation(Field.TILE_WIDTH, 0, RobotConfig.SUPER_CAUTIOUS_SPEED, robot.getDriveTrain(), robot.getIntake(), "Get freight"));

        state.addSecondaryOperation(new WaitOperation(250, "Wait quarter sec before retracting arm"));
        state.addSecondaryOperation(new OutputOperation(robot.getOutPutter(), robot.getIntake(), OutputOperation.Type.Level_Intake, "Intake position"));
        state.addSecondaryOperation(new IntakeOperation(robot.getIntake(), IntakeOperation.Type.Intake, "Intake On"));
        states.add(state);
    }

    private void deliverSecondFreight() {
        State state;
        state = new State("Deliver first freight from warehouse");
        //state.addPrimaryOperation(new FollowTrajectory(field.getReturnToWallTrajectory(), robot.getDriveTrain(), "Reach wall from warehouse"));
        state.addPrimaryOperation(new DriveForDynamicDistanceOperation(true, Match.getInstance().getAlliance() == Alliance.Color.RED ? 45 : -45, RobotConfig.REGULAR_SPEED, robot.getDriveTrain(), "Exit warehouse first time"));
        state.addPrimaryOperation(new BearingOperation(0, robot.getDriveTrain(), "Realign"));
        state.addPrimaryOperation(new DriveInDirectionOperation(-FIRST_ENTRY_DISTANCE, 0, RobotConfig.REGULAR_SPEED, robot.getDriveTrain(), "Complete warehouse exit"));
        state.addPrimaryOperation(new FollowTrajectory(field.getReturnToHubTrajectory(), robot.getDriveTrain(), "Reach hub second time"));
        state.addPrimaryOperation(new OutputOperation(robot.getOutPutter(), robot.getIntake(), OutputOperation.Type.Open, "Deliver second freight"));

        state.addSecondaryOperation(new WaitOperation(1500, "Wait half a sec before raising arm"));
        state.addSecondaryOperation(new OutputOperation(robot.getOutPutter(), robot.getIntake(), OutputOperation.Type.Level_High, "Level High"));
        states.add(state);
    }

    private void enterWarehouseSecondTime() {
        State state;
        state = new State("Get to warehouse second time");
        state.addPrimaryOperation(new FollowTrajectory(field.getSubsequentTimeReachWallTrajectory(), robot.getDriveTrain(), "Reach wall second time"));
        state.addPrimaryOperation(new DriveInDirectionOperation(SECOND_ENTRY_DISTANCE, 0, RobotConfig.REGULAR_SPEED, robot.getDriveTrain(), "Enter warehouse second time"));
        state.addPrimaryOperation(new DriveUntilFreightOperation(Field.TILE_WIDTH, 0, RobotConfig.SUPER_CAUTIOUS_SPEED, robot.getDriveTrain(), robot.getIntake(), "Get freight again"));

        state.addSecondaryOperation(new WaitOperation(250, "Wait quarter sec before retracting arm"));
        state.addSecondaryOperation(new OutputOperation(robot.getOutPutter(), robot.getIntake(), OutputOperation.Type.Level_Intake, "Intake position"));
        state.addSecondaryOperation(new IntakeOperation(robot.getIntake(), IntakeOperation.Type.Intake, "Intake On"));
        states.add(state);
    }

    private void deliverThirdFreight() {
        State state;
        state = new State("Deliver second freight from warehouse");

        state.addPrimaryOperation(new DriveForDynamicDistanceOperation(true, 0, RobotConfig.REGULAR_SPEED, robot.getDriveTrain(), "Exit warehouse second time"));
        state.addPrimaryOperation(new DriveInDirectionOperation(-SECOND_ENTRY_DISTANCE, 0, RobotConfig.REGULAR_SPEED, robot.getDriveTrain(), "Complete warehouse exit"));
        //state.addPrimaryOperation(new FollowTrajectory(field.getReturnToWallTrajectory(), robot.getDriveTrain(), "Reach wall from warehouse"));
        state.addPrimaryOperation(new FollowTrajectory(field.getReturnToHubTrajectory(), robot.getDriveTrain(), "Reach hub third time"));
        state.addPrimaryOperation(new OutputOperation(robot.getOutPutter(), robot.getIntake(), OutputOperation.Type.Open, "Deliver third freight"));

        state.addSecondaryOperation(new WaitOperation(1500, "Wait half a sec before raising arm"));
        state.addSecondaryOperation(new OutputOperation(robot.getOutPutter(), robot.getIntake(), OutputOperation.Type.Level_High, "Level High"));
        states.add(state);
    }

    private void enterWarehouseThirdTime() {
        State state;
        state = new State("Get to warehouse third time");
        state.addPrimaryOperation(new FollowTrajectory(field.getSubsequentTimeReachWallTrajectory(), robot.getDriveTrain(), "Reach wall third time"));
        //state.addPrimaryOperation(new FollowTrajectory(field.getSubsequentNavigateTrajectory(), robot.getDriveTrain(), "Reach warehouse second time"));
        state.addPrimaryOperation(new DriveInDirectionOperation(SECOND_ENTRY_DISTANCE, 0, RobotConfig.REGULAR_SPEED, robot.getDriveTrain(), "Enter warehouse third time"));

        state.addSecondaryOperation(new WaitOperation(500, "Wait half a sec before retracting arm"));
        state.addSecondaryOperation(new OutputOperation(robot.getOutPutter(), robot.getIntake(), OutputOperation.Type.Level_Intake, "Intake position"));
        state.addSecondaryOperation(new IntakeOperation(robot.getIntake(), IntakeOperation.Type.Intake, "Intake On"));
        states.add(state);
    }
}
