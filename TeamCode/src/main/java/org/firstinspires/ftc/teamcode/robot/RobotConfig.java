package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.game.Field;

public class RobotConfig {
    //drive train motors
    public static final String LEFT_FRONT_DRIVE = "leftFrontDrive";
    public static final String LEFT_REAR_DRIVE = "leftRearDrive";
    public static final String RIGHT_REAR_DRIVE = "rightRearDrive";
    public static final String RIGHT_FRONT_DRIVE = "rightFrontDrive";

    public static final String CAROUSEL_MOTOR = "carousel";

    //camera servo
    public static final String CAMERA_SERVO = "cameraServo";

    //webcam id
    public static final String WEBCAM_ID = "Webcam 1";
    public static final double WIDTH = 17.5* Field.MM_PER_INCH;
    public static final double LENGTH = 17.5* Field.MM_PER_INCH;
    public static final double ALLOWED_BEARING_ERROR = 0.5;
    public static final double ALLOWED_POSITIONAL_ERROR = .25;
    public static final double SUPER_CAUTIOUS_SPEED = 0.2;
    public static final double REGULAR_SPEED = 0.6;
    public static final String ARM_MOTOR = "armMotor";
    public static final int ARM_MOTOR_INCREMENT = 10;
    public static final double ARM_SERVO_INCREMENT = .01;
    public static final String ARM_SERVO = "armServo";

    public static final double MAX_CAROUSEL_SPEED = 0.6;
    public static final String BLINKIN = "blinkin";
    public static final long CAROUSEL_SPINNER_REQUIRED_TIME = 2000; //2 seconds to spin carousel for delivery
    public static final String INOUT_TAKE_MOTOR = "inoutMotor";
    public static final double MAX_INOUT_SPEED = 1.0;
}
