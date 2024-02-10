package frc.lib.Constants;

import edu.wpi.first.math.geometry.Translation2d;

public class DriveConstants {
    public static final int FRONT_LEFT_DRIVE_ID = 5;
        public static final int FRONT_LEFT_TURN_ID = 6;
        public static final int FRONT_LEFT_CANCODER_ID = 4;

        public static final int FRONT_RIGHT_DRIVE_ID = 2;
        public static final int FRONT_RIGHT_TURN_ID = 1;
        public static final int FRONT_RIGHT_CANCODER_ID = 1;
        
        public static final int BACK_LEFT_DRIVE_ID = 8;
        public static final int BACK_LEFT_TURN_ID = 7;
        public static final int BACK_LEFT_CANCODER_ID = 3;

        public static final int BACK_RIGHT_DRIVE_ID = 4;
        public static final int BACK_RIGHT_TURN_ID = 3;
        public static final int BACK_RIGHT_CANCODER_ID = 2;

        public static final Translation2d FRONT_LEFT_POS = new Translation2d(0.25,0.25);
        public static final Translation2d FRONT_RIGHT_POS = new Translation2d(0.25,-0.25);
        public static final Translation2d BACK_LEFT_POS = new Translation2d(-0.25,0.25);
        public static final Translation2d BACK_RIGHT_POS = new Translation2d(-0.25,-0.25);

        public static final double TRANSLATIONAL_KP = 0.11525;
        public static final double TRANSLATIONAL_KI = 0;
        public static final double TRANSLATIONAL_KD = 0;
        public static final double TRANSLATIONAL_KS = 0.11885;
        public static final double TRANSLATIONAL_KV = 2.4592;
        public static final double TRANSLATIONAL_KA = 0.10492;

        public static final double ROTATIONAL_KP = 4.0578;
        public static final double ROTATIONAL_KI = 0;
        public static final double ROTATIONAL_KD = 0.1499;
        public static final double ROTATIONAL_KS = 0.17645;
        public static final double ROTATIONAL_KV = 0.42596;
        public static final double ROTATIONAL_KA = 0.0063124;

        public static final double HEADING_KP = 0.1;
        public static final double HEADING_KI = 0;
        public static final double HEADING_KD = 0.0025*5;

        public static final double TRANSLATIONAL_MAX_SPEED = 5;
        public static final double ROTATIONAL_MAX_SPEED = 3.4*(TRANSLATIONAL_MAX_SPEED/5)*Math.PI/2;

        public static final double WHEEL_RADIUS = 0.0508;

        public static final double DRIVE_CONVERSION_FACTOR = 2 * Math.PI * WHEEL_RADIUS / (6.75);
        public static final double TURN_CONVERSION_FACTOR = 2 * Math.PI / (150.0/7.0);
}
