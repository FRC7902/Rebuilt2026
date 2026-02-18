package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;

public final class Constants {

    public static class ClimbConstants {

        // TODO: Most of these are not confirmed, Must be tuned

        // CAN IDs
        public static final int CLIMBER_LEADER_MOTOR_CAN_ID = 1; 
        public static final int CLIMBER_FOLLOWER_MOTOR_CAN_ID = 2; 
        public static final int TONGUE_MOTOR_CAN_ID = 3; 

        // Current Limits
        public static final double CLIMBER_LEADER_STATOR_CURRENT_LIMIT = 40.0; 
        public static final double CLIMBER_FOLLOWER_STATOR_CURRENT_LIMIT = 40.0; 
        public static final double TONGUE_STATOR_CURRENT_LIMIT = 40.0; 
        public static final double ELEVATOR_STATOR_CURRENT_LIMIT = 40.0;

        public static final double TONGUE_SUPPLY_CURRENT_LIMIT = 40.0; 
        public static final double ELEVATOR_SUPPLY_CURRENT_LIMIT = 40.0;

        // PID Constants
        public static final double ELEVATOR_kP = 4.0;
        public static final double ELEVATOR_kI = 0.0;
        public static final double ELEVATOR_kD = 0.0;

        public static final double ELEVATOR_kS = 0.0; 
        public static final double ELEVATOR_kG = 0.0;
        public static final double ELEVATOR_kV = 0.0;

        public static final double TONGUE_kP = 4.0;
        public static final double TONGUE_kI = 0.0;
        public static final double TONGUE_kD = 0.0;

        public static final double TONGUE_kS = 0.0;
        public static final double TONGUE_kG = 0.0;
        public static final double TONGUE_kV = 0.0;

        // Elevator Setpoints
        public static final Distance MAX_HEIGHT = Inches.of(13.125);
        public static final Distance SETPOINT_1 = Inches.of(1.5);
        public static final Distance SETPOINT_2 = Inches.of(6);
        public static final Distance SETPOINT_3 = Inches.of(11);

        //Elevator 
        public static final Distance ELEVATOR_STARTING_HEIGHT = Inches.of(0);
        public static final Distance ELEVATOR_MIN_HEIGHT = Inches.of(0);
        public static final double ELEVATOR_MASS_LBS = 16.0; 

        //Tongue 
        public static final Distance MAX_TONGUE_LENGTH = Inches.of(5);
        public static final Distance MIN_TONGUE_LENGTH = Inches.of(0);
        public static final double TONGUE_MASS_LBS = 5.0; 
    }
}