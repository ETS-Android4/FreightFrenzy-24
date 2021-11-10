package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import org.firstinspires.ftc.teamcode.game.Alliance;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.robot.operations.DriveForTimeOperation;
import org.firstinspires.ftc.teamcode.robot.operations.FollowTrajectory;
import org.firstinspires.ftc.teamcode.robot.operations.State;
import org.firstinspires.ftc.teamcode.robot.operations.WaitOperation;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="RedRight", group="Phoebe")
public class RedRight extends AutonomousHelper {
    @Override
    public void init() {
        super.init(Alliance.Color.RED, Field.StartingPosition.RIGHT);
    }
    @Override
    public void init_loop() {
        super.init_loop();
        if (Field.isInitialized() && !statesAdded) {
            State state;
            //Create a state to get to the alliance shipping hub and deliver box
            state = new State("Deliver box");
            state.addPrimaryOperation(new FollowTrajectory(
                    Field.reachHubTrajectory,
                    robot.getDriveTrain(),"Reach hub"));
            state.addPrimaryOperation(new DriveForTimeOperation(200, 0, 0.3, robot.getDriveTrain(), "Approach Carousel"));
            //TODO - replace this with appropriate code to drop box at the right level
            state.addPrimaryOperation(new WaitOperation(3000, "Wait a sec"));
            states.add(state);

            //Create a state to get to the alliance storage unit
            state = new State("NavigateToWarehouse");
            state.addPrimaryOperation(new FollowTrajectory(
                    Field.navigateTrajectory,
                    robot.getDriveTrain(),"Navigate"));
            states.add(state);

            statesAdded = true;
        }
    }
}
