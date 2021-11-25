package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.game.Field;

public class RobotConfig {
    //drive train motors
    public static final String LEFT_FRONT_DRIVE = "leftFrontDrive";
    public static final String LEFT_REAR_DRIVE = "leftRearDrive";
    public static final String RIGHT_REAR_DRIVE = "rightRearDrive";
    public static final String RIGHT_FRONT_DRIVE = "rightFrontDrive";
    public static final String CAROUSEL_MOTOR = "carousel";
    public static final String WEBCAM_ID = "Webcam 1";
    public static final String BLINKIN = "blinkin";

    public static final String IN_MOTOR = "inMotor";
    public static final String IN_SERVO = "inServo";

    public static final String OUT_SHOULDER = "outShoulder";
    public static final String OUT_ELBOW = "outElbow";
    public static final String OUT_GRIPPER = "outGripper";

    public static final double ROBOT_WIDTH = 17.5* Field.MM_PER_INCH;
    public static final double ROBOT_LENGTH = 17.5* Field.MM_PER_INCH;
    public static final double ALLOWED_BEARING_ERROR = 0.5;
    public static final double ALLOWED_POSITIONAL_ERROR = .25;
    public static final double SUPER_CAUTIOUS_SPEED = 0.2;
    public static final double REGULAR_SPEED = 0.6;

    public static final double INTAKE_SPEED = 1.0;
    public static final double INTAKE_SERVO_INCREMENT = .001;
    public static final double INTAKE_LOWERED_POSITION = .542;
    public static final double INTAKE_RAISED_POSITION = .213;
    public static final double MAX_IN_SPEED = 1.0;

    public static final double ARM_SERVO_INCREMENT = .01;

    public static final double MAX_CAROUSEL_SPEED = 0.1;
    public static final long CAROUSEL_SPINNER_REQUIRED_TIME = 3000; //3 seconds to spin carousel for delivery

    public static final double MAX_OUT_SPEED = 1.0;
    public static final double OUT_ALIGNER_SERVO_INCREMENT = .001;
    public static final double OUT_ALIGNER_FOLDED_POSITION = 0;
    public static final double OUT_ALIGNER_TOP_POSITION = 0.218;
    public static final double OUT_ALIGNER_MIDDLE_POSITION = 0.386;
    public static final double OUT_ALIGNER_BOTTOM_POSITION = 0.53;
    public static final long OUTPUT_LEVEL_REQUIRED_TIME = 300; //300 milli-seconds to reach level
    public static final int DELIVERY_ENCODER_VALUE = 527;
    public static final int INTAKE_ENCODER_VALUE = 0;
}
