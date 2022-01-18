package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.teamcode.robot.RobotConfig;
import org.firstinspires.ftc.teamcode.robot.components.Intake;

import java.util.Date;
import java.util.Locale;

/**
 * Created by Silver Titans on 10/12/17.
 */

public class IntakeOperation extends Operation {
    Intake intake;
    Type type;

    public enum Type {
        Intake,
        Consume,
        Expel
    }

    public IntakeOperation(Intake intake, Type type, String title) {
        this.intake = intake;
        this.type = type;
        this.title = title;
    }

    public String toString() {
        return String.format(Locale.getDefault(), "Intake: Type: %s --%s",
                this.type,
                this.title);
    }

    public boolean isComplete() {
        if (new Date().getTime() - getStartTime().getTime() > RobotConfig.SERVO_REQUIRED_TIME) {
            return true;
        }
        return false;
    }

    @Override
    public void startOperation() {
        switch (type) {
            case Intake: {
                intake.setForIntake();
                break;
            }
            case Consume: {
                intake.setForOutput();
                intake.setInterruption(false);
                intake.setSpeed(0);
                break;
            }
            case Expel: {
                intake.setForExpelling();
                intake.setInterruption(false);
                intake.setSpeed(-RobotConfig.INTAKE_SPEED);
                break;
            }
        }
    }

    @Override
    public void abortOperation() {
        intake.stop();
    }
}

