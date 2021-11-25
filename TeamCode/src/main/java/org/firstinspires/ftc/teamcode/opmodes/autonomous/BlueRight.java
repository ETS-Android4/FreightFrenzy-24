package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import org.firstinspires.ftc.teamcode.game.Alliance;
import org.firstinspires.ftc.teamcode.game.Field;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="BlueRight", group="Phoebe")
public class BlueRight extends NearCarouselAutonomous {
    @Override
    public void init() {
        super.init(Alliance.Color.BLUE, Field.StartingPosition.Right);
    }
}
