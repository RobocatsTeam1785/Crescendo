package frc.lib.Constants;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.Dictionary;
import java.util.Hashtable;

public class DriveConstants {
    public static final int FRONT_LEFT_DRIVE_ID = 1;
        public static final int FRONT_LEFT_TURN_ID = 2;
        public static final int FRONT_LEFT_CANCODER_ID = 3;

        public static final int FRONT_RIGHT_DRIVE_ID = 3;
        public static final int FRONT_RIGHT_TURN_ID = 4;
        public static final int FRONT_RIGHT_CANCODER_ID = 1;
        
        public static final int BACK_LEFT_DRIVE_ID = 5;
        public static final int BACK_LEFT_TURN_ID = 6;
        public static final int BACK_LEFT_CANCODER_ID = 2;

        public static final int BACK_RIGHT_DRIVE_ID = 8;
        public static final int BACK_RIGHT_TURN_ID = 7;
        public static final int BACK_RIGHT_CANCODER_ID = 4;

        public static final Translation2d FRONT_LEFT_POS = new Translation2d(0.25,0.25);
        public static final Translation2d FRONT_RIGHT_POS = new Translation2d(0.25,-0.25);
        public static final Translation2d BACK_LEFT_POS = new Translation2d(-0.25,0.25);
        public static final Translation2d BACK_RIGHT_POS = new Translation2d(-0.25,-0.25);

        public static final double TRANSLATIONAL_KP = 0.044952;
        public static final double TRANSLATIONAL_KI = 0;
        public static final double TRANSLATIONAL_KD = 0;
        public static final double TRANSLATIONAL_KS = 0.18314;
        public static final double TRANSLATIONAL_KV = 2.685;
        public static final double TRANSLATIONAL_KA = 0.50322;

        public static final double ROTATIONAL_KP = 4.7414;
        public static final double ROTATIONAL_KI = 0;
        public static final double ROTATIONAL_KD = 0.49324 * 0;
        public static final double ROTATIONAL_KS = 0.23798;
        public static final double ROTATIONAL_KV = 2.7296;
        public static final double ROTATIONAL_KA = 0.3429;

        public static final double HEADING_KP = 0.1;
        public static final double HEADING_KI = 0;
        public static final double HEADING_KD = 0.01;

        public static final double TRANSLATIONAL_MAX_SPEED = 6;
        public static final double ROTATIONAL_MAX_SPEED = 3.4*(TRANSLATIONAL_MAX_SPEED/5)*Math.PI/2;

        public static final double WHEEL_RADIUS = 0.0508;

        public static final double DRIVE_CONVERSION_FACTOR = 2 * Math.PI * WHEEL_RADIUS / (6.75);
        public static final double TURN_CONVERSION_FACTOR = 2 * Math.PI / (150.0/7.0);

        public static final String AUTO_NAME = "Copy of Four Note Auto Upper";

        public static final Dictionary<String, Double> AUTO_GYRO_POSITIONS = new Hashtable<String, Double>() {{
            put("Four Note Auto Close",180.0);
        }};
}
