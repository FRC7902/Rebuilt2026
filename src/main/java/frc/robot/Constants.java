// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Constants {
    public static class ClimbConstants {
        // elevator setpoints
        public static double MAX_HEIGHT = Units.inchesToMeters(13.125);
        public static double SETPOINT_3 = Units.inchesToMeters(10); //TODO: Must be changed
        public static double SETPOINT_2 = Units.inchesToMeters(5); //TODO: Must be changed
        public static double SETPOINT_1 = Units.inchesToMeters(2); //TODO: Must be changed
        public static double ELEVATOR_BOTTOM = Units.inchesToMeters(0);
        // tongue setpoints
        public static double TONGUE_INITIAL = Units.inchesToMeters(0);
        public static double TONGUE_FULL_EXTENSION = Units.inchesToMeters(3.75);
        // tolerances
        public static double ELEVATOR_TOLERANCE = 0.01;
        public static double TONGUE_TOLERANCE = 0.03;
        // dutycycle constants
        public static double DUTY_CYCLE_ELV = 0.1;
        public static double DUTY_CYCLE_TONGUE = 0.1;
        // CAN ids
        public static int LEADER_MOTOR_CAN_ID = 4;
        public static int FOLLOWER_MOTOR_CAN_ID = 5;
        public static int TONGUE_MOTOR_CAN_ID = 8;
        // PID Constants
        public static double Elevator_kP = 4;
        public static double Elevator_kI = 0;
        public static double Elevator_kD = 0;
        // PID Constants
        public static double Tongue_kP = 4;
        public static double Tongue_kI = 0;
        public static double Tongue_kD = 0;

    }
}
