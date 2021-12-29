package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.RobotConfig;
import org.firstinspires.ftc.teamcode.robot.components.Intake;
import org.firstinspires.ftc.teamcode.robot.components.OutPutter;

import java.util.Date;
import java.util.Locale;

/**
 * Created by Silver Titans on 10/12/17.
 */

public class OutputOperation extends Operation {
    OutPutter outputter;
    Intake intake;
    Type type;

    public enum Type {
        Level_Initial,
        Level_Initial_Release,
        Level_Intake,
        Level_Low,
        Level_Middle,
        Level_High,
        Open,
        Close
    }

    public OutputOperation(OutPutter outputter, Intake intake, Type type, String title) {
        this.outputter = outputter;
        this.intake = intake;
        this.type = type;
        this.title = title;
    }

    public String toString() {
        return String.format(Locale.getDefault(), "Output: Type: %s --%s",
                this.type,
                this.title);
    }

    public boolean isComplete() {
        long currentTime = new Date().getTime();
        switch (type) {
            case Open:
            case Close:
                //for open/close operations: just wait servo turn time to say operation is complete
                return currentTime - getStartTime().getTime() > RobotConfig.SERVO_REQUIRED_TIME;
            case Level_Initial:
            case Level_Initial_Release:
            case Level_Intake:
            case Level_Low:
            case Level_Middle:
            case Level_High: {
                if (outputter.withinReach()) {
                    //get ready to intake if the position specified is for intake
                    if (type == Type.Level_Intake) {
                        outputter.open();
                    }
                    return true;
                }
            }
        }
        return false;
    }

    private void setShoulderPosition() {
        switch (type) {
            case Level_Initial: {
                outputter.setShoulderPosition(RobotConfig.OUTPUT_SHOULDER_INITIAL_POSITION);
                break;
            }
            case Level_High: {
                outputter.setShoulderPosition(RobotConfig.OUTPUT_SHOULDER_TOP_POSITION);
                break;
            }
            case Level_Intake: {
                outputter.setShoulderPosition(RobotConfig.OUTPUT_SHOULDER_INTAKE_POSITION);
                break;
            }
            case Level_Initial_Release:
            case Level_Low: {
                outputter.setShoulderPosition(RobotConfig.OUTPUT_SHOULDER_BOTTOM_POSITION);
                break;
            }
            case Level_Middle: {
                outputter.setShoulderPosition(RobotConfig.OUTPUT_SHOULDER_MIDDLE_POSITION);
                break;
            }
            default : {
                Match.log("Did not change shoulder position for " + type);
            }
        }
        Match.log("Shoulder position. " + outputter.getStatus());

    }
    private void setElbowPosition() {
        switch (type) {
            case Level_Initial: {
                outputter.setElbowPosition(RobotConfig.OUTPUT_ELBOW_INITIAL_POSITION);
                break;
            }
            case Level_Intake: {
                outputter.setElbowPosition(RobotConfig.OUTPUT_ELBOW_INTAKE_POSITION);
                break;
            }
            case Level_Low: {
                outputter.setElbowPosition(RobotConfig.OUTPUT_ELBOW_BOTTOM_POSITION);
                break;                        }
            case Level_Middle: {
                outputter.setElbowPosition(RobotConfig.OUTPUT_ELBOW_MIDDLE_POSITION);
                break;
            }
            case Level_High: {
                outputter.setElbowPosition(RobotConfig.OUTPUT_ELBOW_TOP_POSITION);
                break;
            }
            case Level_Initial_Release: {/*
                //for initial release we set position based on bar code level
                switch (Match.getInstance().getBarcodeLevel()) {
                    case 1: {
                        outputter.setElbowPosition(RobotConfig.OUTPUT_ELBOW_BOTTOM_POSITION);
                        break;
                    }
                    case 2: {
                        outputter.setElbowPosition(RobotConfig.OUTPUT_ELBOW_MIDDLE_POSITION);
                        break;
                    }
                    case 3: {
                        outputter.setElbowPosition(RobotConfig.OUTPUT_ELBOW_TOP_POSITION);
                        break;
                    }
                }*/
                break;
            }
        }
        Match.log("Set elbow position. " + outputter.getStatus());
    }

    @Override
    public void startOperation() {
        switch (type) {
            case Level_Initial_Release:
            case Level_Low:
            case Level_Middle:
            case Level_High:
            case Level_Initial:
            case Level_Intake: {
                //for all bucket positions, we start by closing and folding bucket first
                outputter.close();
                setShoulderPosition();
                setElbowPosition();
                if (type == Type.Level_Intake) {
                    intake.setForIntake();
                }
                break;
            }
            case Open: {
                outputter.open();
                break;
            }
            case Close: {
                outputter.close();
                break;
            }
        }
    }

    @Override
    public void abortOperation() {
        outputter.stop();
    }
}

