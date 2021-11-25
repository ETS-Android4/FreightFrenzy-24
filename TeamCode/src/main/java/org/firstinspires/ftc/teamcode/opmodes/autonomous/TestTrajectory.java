package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.game.Alliance;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.robot.components.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.robot.operations.FollowTrajectory;
import org.firstinspires.ftc.teamcode.robot.operations.State;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="TestTrajectory", group="Phoebe")
public class TestTrajectory extends AutonomousHelper {
    @Override
    public void init() {
        super.init(Alliance.Color.RED, Field.StartingPosition.Right);
        //Create a state to navigate to warehouse
        State navigate = new State("Navigate");
        FollowTrajectory followTrajectory = new FollowTrajectory(
                DriveTrain.trajectoryBuilder(field.getStartingPose())
                        .splineToLinearHeading
                                (field.getStartingPose().plus(new Pose2d(-12,12,45)), 0)
                        .build(),
                robot.getDriveTrain(), "Forward");
        navigate.addPrimaryOperation(followTrajectory);
        states.add(navigate);
    }
}
