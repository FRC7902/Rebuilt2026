package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;

public final class Constants {

    public static class ClimbConstants {

        public static class ElevatorConstants {

            // TODO: Most of these are not confirmed, must be tuned

            // CAN IDs
            public static final int LEADER_MOTOR_CAN_ID = 1;
            public static final int FOLLOWER_MOTOR_CAN_ID = 2;

            // Current Limits
            public static final double STATOR_CURRENT_LIMIT = 40.0;
            public static final double SUPPLY_CURRENT_LIMIT = 40.0;

            // PID Constants
            public static final double kP = 0.1;
            public static final double kI = 0;
            public static final double kD = 0;

            // Feedforward Constants
            public static final double kS = 5.0;
            public static final double kG = 5.0;
            public static final double kV = 5.0;

            // Setpoints
            public static final Distance MAX_HEIGHT = Inches.of(13.125);
            public static final Distance SETPOINT_1 = Inches.of(1.5);
            public static final Distance SETPOINT_2 = Inches.of(6);
            public static final Distance SETPOINT_3 = Inches.of(11);
            public static final Distance TOLERANCE = Meters.of(0.01);

            // Mechanism Config
            public static final Distance STARTING_HEIGHT = Inches.of(0);
            public static final Distance MIN_HEIGHT = Inches.of(0);
            public static final double MASS_LBS = 16.0;
        }

        public static class TongueConstants {

            // TODO: Most of these are not confirmed, must be tuned

            // CAN ID
            public static final int MOTOR_CAN_ID = 3;

            // Current Limits
            public static final double STATOR_CURRENT_LIMIT = 40.0;
            public static final double SUPPLY_CURRENT_LIMIT = 40.0;

            // PID Constants
            public static final double kP = 4.0;
            public static final double kI = 5.0;
            public static final double kD = 5.0;

            // Feedforward Constants
            public static final double kS = 5.0;
            public static final double kG = 5.0;
            public static final double kV = 5.0;

            // Setpoints
            public static final Distance MAX_LENGTH = Inches.of(5);
            public static final Distance MIN_LENGTH = Inches.of(0);

            // Mechanism Config
            public static final double MASS_LBS = 5.0;
        }
    }
}