// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;

/** Add your docs here. */
public class Constants {
    public static class IntakeConstants {
        // elevator setpoints
        public static Distance RETRACT_SETPOINT = Meters.of(Units.inchesToMeters(0));
        public static Distance EXTEND_SETPOINT = Meters.of(Units.inchesToMeters(16.325));
        public static double DEFAULT_POSITION = Units.inchesToMeters(0); //TODO: Must change this value perhaps
        // limit switch constants
        public static int SHALLOW_BUTTON_BREAK_DIO = 1;
        public static int DEEP_BUTTON_BREAK_DIO = 2;
        // roller constants
        public static double INTAKE_SPEED = 1;
        public static double OUTAKE_SPEED = -1;
        // tolerances
        public static double LINEAR_TOLERANCE = 0.01;
        public static double TONGUE_TOLERANCE = 0.03;
        // dutycycle constants
        public static double DUTY_CYCLE_ELV = 0.1;
        // CAN ids
        public static int LINEAR_MOTOR_CAN_ID = 4;
        public static int ROLLER_MOTOR_CAN_ID = 8;
        // PID Constants Linear Actuator
        public static double Linear_kP = 4;
        public static double Linear_kI = 0;
        public static double Linear_kD = 0;
        // Feedfoward Constants Linear Actuator
        public static double Linear_kS = 0;
        public static double Linear_kG = 0;
        public static double Linear_kV = 0;

    }
}
