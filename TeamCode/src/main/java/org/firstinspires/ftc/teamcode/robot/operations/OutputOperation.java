package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.teamcode.robot.RobotConfig;
import org.firstinspires.ftc.teamcode.robot.components.OutPutter;

import java.util.Date;
import java.util.Locale;

/**
 * Created by Silver Titans on 10/12/17.
 */

public class OutputOperation extends Operation {
    OutPutter outputter;
    Type type;

    public enum Type {
        Level_Low,
        Level_Middle,
        Level_High,
        Deliver,
        Retract,
        Fold
    }

    public OutputOperation(OutPutter outputter, Type type, String title) {
        this.outputter = outputter;
        this.type = type;
        this.title = title;
    }

    public String toString() {
        return String.format(Locale.getDefault(), "Output: Type: %s --%s",
                this.type,
                this.title);
    }

    public boolean isComplete() {
        if (type == Type.Level_Low || type == Type.Level_Middle || type == Type.Level_High
            || type == Type.Fold) {
            if (new Date().getTime() - getStartTime().getTime() > RobotConfig.OUTPUT_LEVEL_REQUIRED_TIME) {
                return true;
            }
            return false;
        }
        else {
            return outputter.withinReach();
        }
    }

    @Override
    public void startOperation() {
        switch (type) {
            case Level_Low: {
                outputter.setLowPosition();
                break;
            }
            case Level_Middle: {
                outputter.setMiddlePosition();
                break;
            }
            case Level_High: {
                outputter.setHighPosition();
                break;
            }
            case Fold: {
                outputter.fold();
                break;
            }
            case Deliver: {
                outputter.deliver();
                break;
            }
            case Retract: {
                outputter.retract();
                break;
            }
        }
    }

    @Override
    public void abortOperation() {
        outputter.stop();
    }
}

