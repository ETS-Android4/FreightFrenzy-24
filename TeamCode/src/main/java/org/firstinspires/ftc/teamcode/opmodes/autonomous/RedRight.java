package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import org.firstinspires.ftc.teamcode.game.Alliance;
import org.firstinspires.ftc.teamcode.game.Field;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="RedRight", group="Phoebe")
public class RedRight extends NearWarehouseAutonomous {
    @Override
    public void init() {
        super.init(Alliance.Color.RED, Field.StartingPosition.Right);
    }
}
