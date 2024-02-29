package frc.lib.Constants;

public class ShooterConstants {
    public static final int TOP_MOTOR_ID = 15;
    public static final int BOTTOM_MOTOR_ID = 16;

    public static final double TOP_KP = 0.00015675;
    public static final double TOP_KI = 0;
    public static final double TOP_KD = 0;
    public static final double TOP_KS = 0.18515*0;
    public static final double TOP_KV = 0.13000;//0.13197
    public static final double TOP_KA = 0.014356*0;

    public static final double BOTTOM_KP = 0.0082625;
    public static final double BOTTOM_KI = 0;
    public static final double BOTTOM_KD = 0;
    public static final double BOTTOM_KS = 0.08264*0;
    public static final double BOTTOM_KV = 0.125;//12648
    public static final double BOTTOM_KA = 0.018314*0;

    public static final double MAX_RPM_TOP = 4500/60;
    public static final double MAX_RPM_BOTTOM = 4500/60;
    public static final double DEFAULT_RPM = 1500/60;

    public static final double ENCODER_CONVERSION_FACTOR = 1;

    public static final double MARGIN_OF_ERROR = 75/60;
}
