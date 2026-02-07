package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static class DriverOperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1; // Tentative, there may not be an operator
        public static final double DEADBAND = 0.1;
    }
    public static class SwerveConstants {
        public static final double MAX_SPEED = 6; // To be tuned
    }
    public static class VisionConstants {
        public static final String APRILTAGS_LL4_NAME = "limelight-a";
        public static final String APRILTAGS_LL3G_NAME = "limelight-b";
        public static final String OBJECT_DETECTION_LL4_NAME = "limelight-c";

        public static final double ROLL_PITCH_MT2_DISABLE_THRESHOLD = 2;

        public static final double VELOCITY_COMPENSATOR_COEFFICIENT = 0.5; // To be tuned
    }
    public static class PositionConstants   {
        public static final Translation2d RED_HUB_T_2D = new Translation2d(4.0218614 + Units.inchesToMeters(47.0) / 2.0, 8.069 / 2);
        public static final Translation2d BLUE_HUB_T_2D = new Translation2d(0, 0); // TBD
    }
}
