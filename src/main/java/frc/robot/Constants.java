package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {
    
    public static class ClimbConstants {
        
        // CAN IDs
        public static final int CLIMBER_LEFT_MOTOR_CAN_ID = 1; //TODO: Update this
        public static final int CLIMBER_RIGHT_MOTOR_CAN_ID = 1; //TODO: Update this
        
        // Current Limits
        public static final double LEFT_MOTOR_STATOR_CURRENT_LIMIT = 40.0; //TODO: Update this
        public static final double LEFT_MOTOR_SUPPLY_CURRENT_LIMIT = 40.0; //TODO: Update this
        public static final double RIGHT_MOTOR_STATOR_CURRENT_LIMIT = 40.0; //TODO: Update this
        public static final double RIGHT_MOTOR_SUPPLY_CURRENT_LIMIT = 40.0; //TODO: Update this
        
        // Target Heights
        public static final double kLevel1Height = Units.inchesToMeters(27.0);
        public static final double kLevel2Height = Units.inchesToMeters(54.0); // TENTATIVE
        public static final double kLevel3Height = Units.inchesToMeters(81.0); // TENTATIVE
        
        // Control Parameters
        public static final double kPositionTolerance = 0.01;
        public static final double kHoldOutput = 0.2;
        public static final double kMaxOutput = 0.8;
        public static final double kMaxClimbTime = 30.0;
        
        // Mechanism Conversion
        public static final double kHeightToRotationsConversion = 10.0;
        
        // PID Constants
        public static final double kP = 0.1;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kF = 0.0;
        
        // Safety Limits
        public static final double kMinHeight = 0.0;
        public static final double kMaxHeight = Units.inchesToMeters(100.0);
    }
}