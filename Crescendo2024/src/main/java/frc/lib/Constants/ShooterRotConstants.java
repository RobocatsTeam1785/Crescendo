package frc.lib.Constants;

public class ShooterRotConstants {
    public static final int MOTOR_ID = 13;

    public static final int SHAFT_ENCODER_ID = 0;
    public static final double SHAFT_CONVERSION_FACTOR = 1;

    public static final double KP_VALUE = 8;
    public static final double KI_VALUE = 0;
    public static final double KD_VALUE = 0.3;
    public static final double KS_VALUE = 0.2;
    public static final double KG_VALUE = 0.1;
    public static final double KV_VALUE = 1.1;
    public static final double KA_VALUE = 0;




    public static final double MAX_ROTATIONAL_SPEED = 16 * Math.PI;
    public static final double MAX_ACCELERATION = 16 * Math.PI;

    public static final double INTAKE_ANGLE = (29-90)*Math.PI/180;

    public static final double ROT_OFFSET = 0.884;

    public static final double CLOSE_STAGE_ANGLE = (55-90)*Math.PI/180;

    public static final double PROTECTED_ZONE_ANGLE = (40-90)*Math.PI/180;

    public static final double AMP_ANGLE = (50-90)*Math.PI/180;

}
