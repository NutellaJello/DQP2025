package org.firstinspires.ftc.teamcode.subsystems;

/** Hardware names and tuning shared by every competition teleop and auto. */
public final class RobotConstants {
    private RobotConstants() { }

    public static final String INTAKE_MOTOR = "intake";
    public static final String TURRET_MOTOR = "turret";
    public static final String FLYWHEEL_ONE_MOTOR = "FW1";
    public static final String FLYWHEEL_TWO_MOTOR = "FW2";
    public static final String STOPPER_SERVO = "stopper";
    public static final String FLAP_SERVO = "flap";
    public static final String WEBCAM = "Webcam 1";

    public static final double FLYWHEEL_P = 400;
    public static final double FLYWHEEL_I = 0;
    public static final double FLYWHEEL_D = 0;
    public static final double FLYWHEEL_F = 13.5;
    public static final double TURRET_MIN_TICKS = -1906;
    public static final double TURRET_MAX_TICKS = 340;
    public static final double CAMERA_FORWARD_OFFSET_IN = 2;
    public static final int CAMERA_EXPOSURE_MS = 2;
    public static final int CAMERA_GAIN = 100;
    public static final int RED_GOAL_TAG_ID = 24;
    public static final int BLUE_GOAL_TAG_ID = 20;
}
