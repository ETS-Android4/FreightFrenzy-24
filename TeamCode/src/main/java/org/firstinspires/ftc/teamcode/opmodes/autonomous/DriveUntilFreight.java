package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import org.firstinspires.ftc.teamcode.game.Alliance;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.robot.operations.BearingOperation;
import org.firstinspires.ftc.teamcode.robot.operations.DriveInDirectionOperation;
import org.firstinspires.ftc.teamcode.robot.operations.DriveUntilFreightOperation;
import org.firstinspires.ftc.teamcode.robot.operations.IntakeOperation;
import org.firstinspires.ftc.teamcode.robot.operations.OutputOperation;
import org.firstinspires.ftc.teamcode.robot.operations.State;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="DriveUntilFreight", group="Phoebe")
public class DriveUntilFreight extends AutonomousHelper {
    @Override
    public void init() {
        super.init(Alliance.Color.BLUE, Field.StartingPosition.Left);
    }
    @Override
    public void start() {
        super.start();
        State state;
        state = new State("Intake");

        state.addPrimaryOperation(new OutputOperation(robot.getOutPutter(), robot.getIntake(), OutputOperation.Type.Level_Intake, "Outputter intake position"));
        state.addPrimaryOperation(new IntakeOperation(robot.getIntake(), IntakeOperation.Type.Intake, "Intake On"));
        state.addPrimaryOperation(new BearingOperation(0, robot.getDriveTrain(), "Turn"));
        state.addPrimaryOperation(new DriveUntilFreightOperation(Field.TILE_WIDTH*2, 0, .4, robot.getDriveTrain(), robot.getIntake(), "Drive until freight"));

        state.addPrimaryOperation(new DriveInDirectionOperation(-Field.TILE_WIDTH*2, 0, .4, robot.getDriveTrain(), "Drive back"));
        states.add(state);

    }
}
