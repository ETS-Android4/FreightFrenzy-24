package org.firstinspires.ftc.teamcode.robot.operations;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.RobotConfig;
import org.firstinspires.ftc.teamcode.robot.components.Intake;
import org.firstinspires.ftc.teamcode.robot.components.OutPutter;

import java.util.Date;
import java.util.Locale;

import static org.firstinspires.ftc.teamcode.robot.operations.OutputOperation.Type.Level_Intake;

/**
 * Created by Silver Titans on 10/12/17.
 */

public class OutputOperation extends Operation {
    OutPutter outputter;
    Intake intake;
    Type type;
    Date elbowPositionSetTime;

    public enum Type {
        Level_Initial,
        Level_Intake,
        Level_Low,
        Level_Middle,
        Level_High,
        Level_Llama,
        Level_Pickup,
        Vertical,
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
        if (type == Type.Open || type == Type.Close) {
            //for open/close operations: just wait servo turn time to say operation is complete
            return currentTime - getStartTime().getTime() > RobotConfig.SERVO_REQUIRED_TIME;
        }
        //for operations requiring shoulder and elbow movement
        if (outputter.withinReach()) {
            if (elbowPositionSetTime == null) {
                setElbowPosition();
                elbowPositionSetTime = new Date();
            }
            if (currentTime - elbowPositionSetTime.getTime() > RobotConfig.SERVO_REQUIRED_TIME) {
                if (type == Level_Intake) {
                    outputter.open();
                    outputter.setInIntakePosition(true);
                }
                else if (type == Type.Level_Pickup) {
                    outputter.open();
                }
                Match.getInstance().setLed(RevBlinkinLedDriver.BlinkinPattern.WHITE);
                return true;
            }
        }
        else {
            return false;
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
            case Level_Low: {
                outputter.setShoulderPosition(RobotConfig.OUTPUT_SHOULDER_BOTTOM_POSITION);
                break;
            }
            case Level_Middle: {
                outputter.setShoulderPosition(RobotConfig.OUTPUT_SHOULDER_MIDDLE_POSITION);
                break;
            }
            case Level_Llama: {
                outputter.setShoulderPosition(RobotConfig.OUTPUT_SHOULDER_LLAMA_POSITION);
                break;
            }
            case Level_Pickup: {
                outputter.setShoulderPosition(RobotConfig.OUTPUT_SHOULDER_PICKUP_POSITION);
                break;
            }
            case Vertical: {
                outputter.setShoulderPosition(RobotConfig.OUTPUT_SHOULDER_VERTICAL_POSITION);
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
            case Level_Llama: {
                outputter.setElbowPosition(RobotConfig.OUTPUT_ELBOW_LLAMA_POSITION);
                break;
            }
            case Level_Pickup: {
                outputter.setElbowPosition(RobotConfig.OUTPUT_ELBOW_PICKUP_POSITION);
                break;
            }
            case Vertical: {
                outputter.setElbowPosition(RobotConfig.OUTPUT_ELBOW_VERTICAL_POSITION);
                break;
            }
        }
        Match.log("Set elbow position. " + outputter.getStatus());
    }

    @Override
    public void startOperation() {
        outputter.setInIntakePosition(false);
        switch (type) {
            case Vertical:
            case Level_Pickup:
            case Level_Llama:
            case Level_Low:
            case Level_Middle:
            case Level_High:
            case Level_Initial:
            case Level_Intake: {
                //for all bucket positions, we start by closing bucket first
                outputter.close();
                //then getting elbow to the release position first
                outputter.setElbowPosition(RobotConfig.OUTPUT_ELBOW_INITIAL_POSITION);
                //set the desired shoulder positions
                setShoulderPosition();
                //lower intake so bucket can fit in the intake position
                if (type == Level_Intake) {
                    intake.lowerIntake();
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

