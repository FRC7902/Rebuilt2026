// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import yams.gearing.GearBox;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.units.measure.Mass;

/** Add your docs here. */
public class Constants {

  public static class OperatorConstants {

    // Joystick Deadband
    public static final double DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
  }

  public static class SwerveConstants {
    public static final double MAX_SPEED = Units.feetToMeters(5);

    // Speed scaling factors, should be between 0 and 1
    public static final double MIN_TRANSLATION_SPEED_SCALE = 0.175; // Minimum speed scaling factor for joystick input
    public static final double MIN_ROTATION_SPEED_SCALE = 0.25; // Minimum speed scaling factor for joystick input

    public static final double SLOW_SPEED_DRIVE_RATE = 0.25;
    public static final double RAMP_SPEED_DRIVE_RATE = 0.25;
  }

  public static class PathPlanner {
    public static final double kPDrive = 1.95;
    public static final double kIDrive = 0;
    public static final double kDDrive = 0.01;

    public static final double kPAngle = 2.6;
    public static final double kIAngle = 0;
    public static final double kDAngle = 0.01;

  }

public static class IntakeConstants {
        // elevator setpoints
        public static Distance RETRACT_SETPOINT = Meters.of(Units.inchesToMeters(0));
        public static Distance EXTEND_SETPOINT = Meters.of(Units.inchesToMeters(16.325));
        // limit switch constants
        public static int RETRACTED_LIMIT_SWITCH_DIO = 1;
        public static int EXTENDED_LIMIT_SWITCH_DIO = 2;
        // tolerances
        public static double LINEAR_TOLERANCE = 0.01;
        // dutycycle constants
        public static double DUTY_CYCLE_ELV = 0.1;
        // CAN ids
        public static int LINEAR_MOTOR_CAN_ID = 18;
        // PID Constants Linear Actuator
        public static double Linear_kP = 34.935;
        public static double Linear_kI = 0;
        public static double Linear_kD = 0.9684;

        public static double Sim_kP = 4;
        public static double Sim_kI = 0;
        public static double Sim_kD = 0;

        public static Time CLOSED_LOOP_RAMP_RATE = Seconds.of(0.25);
         public static Time OPEN_LOOP_RAMP_RATE = Seconds.of(0.25);
        // Velocity + Acceleration
        public static final LinearVelocity MAX_VELOCITY = MetersPerSecond.of(1); // TODO
        public static final LinearAcceleration MAX_ACCELERATION = MetersPerSecondPerSecond.of(5); // TODO
        // Feedfoward Constants Linear Actuator
        public static double Linear_kS = 0;
        public static double Linear_kG = 0;
        public static double Linear_kV = 0;
        public static ElevatorFeedforward FEED_FORWARD = new ElevatorFeedforward(0.28669, 0,0.60431, 0.021882); // TODO: kg
        // Current Limits
        public static Current STATOR_CURRENT_LIMIT = Amps.of(10); //TODO: Change this
        // Elevator Config Constants
        public static Distance STARTING_HEIGHT = Meters.of(0.5);
        public static Distance MIN_HARD_LIMIT = Meters.of(0);
        public static Distance MAX_HARD_LIMIT = Meters.of(0.3333);
        public static Mass DLI_MASS = Pounds.of(6.825);
        public static Angle DLI_ANGLE = Degrees.of(360 - 24.16);
        public static Distance MECH_CIRCUMFERENCE = Inches.of(Math.PI);
        public static boolean MOTOR_INVERTED = false;
        // Gear Reduction Constants
        public static final GearBox GEARBOX = GearBox.fromStages("54:16", "18:12");
    }
    public static class RollerConstants {
        public static double INTAKE_SPEED = 1;
        public static double OUTAKE_SPEED = -1;
        public static int ROLLER_MOTOR_CAN_ID = 8;
        // Current Limits
        public static double STATOR_CURRENT_LIMIT = 50.0;
        public static double SUPPLY_CURRENT_LIMIT = 30.0;
    }
}