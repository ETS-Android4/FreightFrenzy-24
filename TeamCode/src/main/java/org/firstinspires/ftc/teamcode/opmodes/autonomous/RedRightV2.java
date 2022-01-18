package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.game.Alliance;
import org.firstinspires.ftc.teamcode.game.Field;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="RedRightV2", group="Phoebe")
@Disabled
public class RedRightV2 extends NearWarehouseAutonomousV2 {
    @Override
    public void init() {
        super.init(Alliance.Color.RED, Field.StartingPosition.Right);
    }
}
