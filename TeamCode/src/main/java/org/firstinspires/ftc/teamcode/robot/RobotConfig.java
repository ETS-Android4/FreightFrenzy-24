package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.game.Field;

public class RobotConfig {
    //drive train motors
    public static final String LEFT_FRONT_DRIVE = "leftFrontDrive";
    public static final String LEFT_REAR_DRIVE = "leftRearDrive";
    public static final String RIGHT_REAR_DRIVE = "rightRearDrive";
    public static final String RIGHT_FRONT_DRIVE = "rightFrontDrive";

    //camera servo
    public static final String CAMERA_SERVO = "cameraServo";

    //webcam id
    public static final String WEBCAM_ID = "Webcam 1";
    public static final double WIDTH = 17.5* Field.MM_PER_INCH;
    public static final double LENGTH = 17.5* Field.MM_PER_INCH;
    public static final double ALLOWED_BEARING_ERROR = Math.toRadians(0.5);
    public static final double ALLOWED_POSITIONAL_ERROR = .25;
    public static final double SUPER_CAUTIOUS_SPEED = 0.2;
    public static final double REGULAR_SPEED = 0.6;
}
