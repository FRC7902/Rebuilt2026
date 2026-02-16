// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Constants {
    public static class ClimbConstants {
        public static double kLEVEL1 = Units.inchesToMeters(27);
        public static double kLEVEL2 = Units.inchesToMeters(45);
        public static double kLEVEl3 = Units.inchesToMeters(63);
        public static double TRAVEL_DISTANCE = Units.inchesToMeters(13.125);
        public static double DISTANCE_BEFORE_TONGUE_EXTENDS = Units.inchesToMeters(9.5); //also known as DTBE
        public static double FROM_DBTE_TO_TRAVEL_DISTANCE  = TRAVEL_DISTANCE - DISTANCE_BEFORE_TONGUE_EXTENDS;
        public static double TONGUE_FULL_EXTENSION = Units.inchesToMeters(12);
        public static double ELEVATOR_TOLERANCE = 0.1;
        public static double TONGUE_TOLERANCE = 0.1;

        public static double ELEVATOR_BOTTOM = Units.inchesToMeters(0);
        public static double TONGUE_INITIAL = Units.inchesToMeters(0);

        public static int LEADER_MOTOR_CAN_ID = 4;
        public static int FOLLOWER_MOTOR_CAN_ID = 5;
        public static int TONGUE_MOTOR_CAN_ID = 8;


    }
}
