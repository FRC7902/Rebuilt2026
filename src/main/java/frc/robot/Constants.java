// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import yams.gearing.GearBox;

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
        // dutycycle constants
        public static double DUTY_CYCLE_ELV = 0.1;
        // CAN ids
        public static int LINEAR_MOTOR_CAN_ID = 18;
        public static int ROLLER_MOTOR_CAN_ID = 8;
        // PID Constants Linear Actuator
        public static double Linear_kP = 34.935;
        public static double Linear_kI = 0;
        public static double Linear_kD = 0.9684;
        public static Time CLOSED_LOOP_RAMP_RATE = Seconds.of(0.25);
         public static Time OPEN_LOOP_RAMP_RATE = Seconds.of(0.25);
        // Velocity + Acceleration
        public static final LinearVelocity MAX_VELOCITY = MetersPerSecond.of(1); // TODO
        public static final LinearAcceleration MAX_ACCELERATION = MetersPerSecondPerSecond.of(5); // TODO
        // Feedfoward Constants Linear Actuator
        public static double Linear_kS = 0;
        public static double Linear_kG = 0;
        public static double Linear_kV = 0;
        // Current Limits
        public static Current STATOR_CURRENT_LIMIT = Amps.of(10); //TODO: Change this
        // Elevator Config Constants
        public static Distance STARTING_HEIGHT = Meters.of(0.5);
        public static Distance MIN_HARD_LIMIT = Meters.of(0);
        public static Distance MAX_HARD_LIMIT = Meters.of(0.3333);
        public static Mass DLI_MASS = Pounds.of(6.825);
        public static Angle DLI_ANGLE = Degrees.of(360 - 24.16);
        public static Distance MECH_CIRCUMFERENCE = Inches.of(Math.PI);

        // Gear Reduction Constants
        public static final GearBox GEARBOX = GearBox.fromStages("54:16", "18:12");
    }
}
