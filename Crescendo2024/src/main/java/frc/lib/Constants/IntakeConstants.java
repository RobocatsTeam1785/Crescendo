package frc.lib.Constants;

public class IntakeConstants {
    public static final int TOP_MOTOR_ID = 9;
    public static final int BOTTOM_MOTOR_ID = 10;

    public static final double TOP_KP = 0.031271*0;
    public static final double TOP_KI = 0;
    public static final double TOP_KD = 0;
    public static final double TOP_KS = 0.2365;
    public static final double TOP_KV = 0.50136*6;
    public static final double TOP_KA = 0.047712*0;

    public static final double BOTTOM_KP = 0.0051744;
    public static final double BOTTOM_KI = 0;
    public static final double BOTTOM_KD = 0;
    public static final double BOTTOM_KS = 0.69848;
    public static final double BOTTOM_KV = 0.28111;
    public static final double BOTTOM_KA = 0.040753;

    public static final double MAX_SPEED_TOP = 38;
    public static final double MAX_SPEED_BOTTOM = 0;
    public static final double MAX_ACCEL_TOP = 0;
    public static final double MAX_ACCEL_BOTTOM = 0;
    public static final double DEFAULT_RPM = 30;

    public static final double ENCODER_CONVERSION_FACTOR = 1/4.0;

    //public static final int PHOTO_ELECTRIC_SENSOR_ID = 0;
}
