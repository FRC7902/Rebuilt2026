// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorControllerConfig;

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
    public static final double MAX_SPEED = Units.feetToMeters(16);

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
    public static Distance RETRACT_SETPOINT = Meters.of(0.015 + 0.02);
    public static Distance MID_SETPOINT = Meters.of(0.154995);
    public static Distance EXTEND_SETPOINT = Meters.of(0.33 - 0.02);
    // limit switch constants
    public static int RETRACTED_LIMIT_SWITCH_DIO = 3;
    public static int EXTENDED_LIMIT_SWITCH_DIO = 4;
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

    public static Time CLOSED_LOOP_RAMP_RATE = Seconds.of(0.1);
    public static Time OPEN_LOOP_RAMP_RATE = Seconds.of(0.1);
    // Velocity + Acceleration
    public static final LinearVelocity MAX_VELOCITY = MetersPerSecond.of(1); // TODO change back to 2
    public static final LinearAcceleration MAX_ACCELERATION = MetersPerSecondPerSecond.of(6.5); // TODO
    // Feedfoward Constants Linear Actuator
    public static double Linear_kS = 0;
    public static double Linear_kG = 0;
    public static double Linear_kV = 0;
    public static ElevatorFeedforward FEED_FORWARD = new ElevatorFeedforward(0.28669, 0, 0.60431, 0.021882); // TODO: kg
    // Current Limits
    public static Current STATOR_CURRENT_LIMIT = Amps.of(40); // TODO: Change this
    public static Current SUPPLY_CURRENT_LIMIT = Amps.of(30); // TODO: Change this
    // Elevator Config Constants
    public static Distance STARTING_HEIGHT = Meters.of(0);
    public static Distance MIN_HARD_LIMIT = Meters.of(0.015);
    public static Distance MAX_HARD_LIMIT = Meters.of(0.33);
    public static Mass DLI_MASS = Pounds.of(6.825);
    public static Angle DLI_ANGLE = Degrees.of(360 - 24.16);
    public static Distance MECH_CIRCUMFERENCE = Inches.of(Math.PI);
    public static boolean MOTOR_INVERTED = true;
    // Gear Reduction Constants
    public static final GearBox GEARBOX = GearBox.fromStages("54:16", "18:12");
  }

  public static class RollerConstants {
    public static double INTAKE_SPEED = 0.75;
    public static double OUTAKE_SPEED = -0.75;
    public static int ROLLER_MOTOR_PWM_ID = 1;
    // Current Limits
    public static double STATOR_CURRENT_LIMIT = 50.0;
    public static double SUPPLY_CURRENT_LIMIT = 30.0;
  }

  public static class IndexerConstants {
    public static final int INDEXER_MOTOR_CAN_ID = 19;
    public static final double AGGRESIVE_MOTOR_SPEED = 0.125;
    public static final boolean TELEMETRY = false;
  }
  public static class FlyWheelConstants {
    /*Flywheel Subsystem*/
		public static final int FLYWHEEL_LEADER_ID = 20;
		public static final int FLYWHEEL_FOLLOWER_ID = 21;
		//TODO: needs to be tuned
		public static final double FLYWHEEL_KP = 0.85;
		public static final double FLYWHEEL_KI = 0;
		public static final double FLYWHEEL_KD = 0;
		public static final AngularVelocity FLYWHEEL_MAX_VELOCITY = RPM.of(11568);
		public static final AngularVelocity FLYWHEEL_TARGET_ANGULAR_VELOCITY = RPM.of(2900);
		public static final AngularAcceleration FLYWHEEL_MAX_ACCELERATION = RotationsPerSecondPerSecond.of(173);
		public static final SmartMotorControllerConfig.MotorMode FLYWHEEL_IDLE = SmartMotorControllerConfig.MotorMode.COAST;
		public static final Current FLYWHEEL_STATOR = Amps.of(80);
		public static final Time FLYWHEEL_CLOSED_RATE = Seconds.of(0.25);
		public static final Time FLYWHEEL_OPEN_RATE = Seconds.of(0.25);
		public static final double FLYWHEEL_KS = 0.52872;
		public static final double FLYWHEEL_KV = 0.062754;
		public static final double FLYWHEEL_KA = 0.013174;

		public static final MechanismGearing FLYWHEEL_GEARING = new MechanismGearing(GearBox.fromStages("1:2"));
		public static final Distance FLYWHEEL_DIAMETER = Inches.of(3);
		public static final MomentOfInertia FLYWHEEL_MOI = KilogramSquareMeters.of(0.0006438072);

  }
  public static class HoodConstants {
    /*Hood Subsystem*/
		public static final int HOOD_ID = 22;
		//TODO: needs to be tuned
		public static final double HOOD_KP = 30.865;
		public static final double HOOD_KI = 0.24656;
		public static final double HOOD_KD = 3.0998;
		public static final double HOOD_SIM_KP = 30.865;
		public static final double HOOD_SIM_KI = 0.24656;
		public static final double HOOD_SIM_KD = 3.0998;
		public static final AngularVelocity HOOD_MAX_VELOCITY = RPM.of(5000);
		public static final AngularAcceleration HOOD_MAX_ACCELERATION = RotationsPerSecondPerSecond.of(2500);
		public static final SmartMotorControllerConfig.MotorMode HOOD_MOTOR_IDLE_MODE = SmartMotorControllerConfig.MotorMode.COAST;
		public static final Current HOOD_STATOR = Amps.of(40);
		public static final Current HOOD_SUPPLY = Amps.of(40);
		public static final Time HOOD_CLOSED_RATE = Seconds.of(0.25);
		public static final Time HOOD_OPEN_RATE = Seconds.of(0.25);
		public static final double HOOD_KS = 0.2417;
		public static final double HOOD_KV = 11.922;
		public static final double HOOD_KA = 0.37754;

		public static final MechanismGearing HOOD_GEARING = new MechanismGearing(GearBox.fromStages("80:14","24:18","170:10"));//new MechanismGearing(GearBox.fromReductionStages((double) 80 /14, (double) 24 /18, (double) 170 /10));
		public static final Distance HOOD_LENGTH = Inches.of(8.884333);
		public static final MomentOfInertia HOOD_MOI = KilogramSquareMeters.of(0.0664243573);
		public static final Angle HOOD_SOFT_LIMIT_LOW = Degrees.of(5.5);
		public static final Angle HOOD_SOFT_LIMIT_HIGH = Degrees.of(43);
		public static final Angle HOOD_HARD_LIMIT_LOW = Degrees.of(0);
		public static final Angle HOOD_START_POSITION = Degrees.of(5.5);
		public static final Angle  HOOD_HARD_LIMIT_HIGH = Degrees.of(45);

  }
  public static class FeederConstants {
    /*Feeder Subsystem*/
		public static final int FEEDER_ID = 23;
		//TODO: Needs to be tuned
		public static final double FEEDER_KP = 40;
		public static final double FEEDER_KI = 10;
		public static final double FEEDER_KD = 1;
		public static final double FEEDER_STATOR = 20;
		public static final double FEEDER_SUPPLY = 20;
		public static final double FEEDER_KV = 1;
		public static final double FEEDER_KA = 1;
		public static final double FEEDER_TARGET_SPEED = 30; //TODO: Must be tested

		public static final double FEEDER_SIM_LENGTH = 0.0762;

		public static final double FEEDER_TIME_PERIOD = 2;
		public static final double FEEDER_GEAR_RATIO = 10;
		public static final double FEEDER_VOLTAGE = 12;

		public static final int BEAM_BREAK_BOTTOM_ID = 5;
		public static final int BEAM_BREAK_TOP_ID = 6;

  }
  public static class ShooterConstants {
    
    // Degree to Displacement mapping
    public static final InterpolatingTreeMap<Double, Rotation2d> hoodAngleMapHub =
			new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
    public static final InterpolatingTreeMap<Double, Rotation2d> hoodAngleMapNeutral =
        new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
    public static final InterpolatingTreeMap<Double, Rotation2d> hoodAngleMapZone =
        new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);

    static {
      hoodAngleMapHub.put(36d, Rotation2d.fromDegrees(Utils.reverse(84.5)));
      hoodAngleMapHub.put(40d, Rotation2d.fromDegrees(Utils.reverse(84.5)));
      hoodAngleMapHub.put(50d, Rotation2d.fromDegrees(Utils.reverse(84.5)));
      hoodAngleMapHub.put(60d, Rotation2d.fromDegrees(Utils.reverse(83.5)));
      hoodAngleMapHub.put(70d, Rotation2d.fromDegrees(Utils.reverse(82.5)));
      hoodAngleMapHub.put(80d, Rotation2d.fromDegrees(Utils.reverse(81.5)));
      hoodAngleMapHub.put(90d, Rotation2d.fromDegrees(Utils.reverse(80)));
      hoodAngleMapHub.put(100d, Rotation2d.fromDegrees(Utils.reverse(79)));
      hoodAngleMapHub.put(110d, Rotation2d.fromDegrees(Utils.reverse(77.5)));
      hoodAngleMapHub.put(120d, Rotation2d.fromDegrees(Utils.reverse(76)));
      hoodAngleMapHub.put(130d, Rotation2d.fromDegrees(Utils.reverse(74.5)));
      hoodAngleMapHub.put(140d, Rotation2d.fromDegrees(Utils.reverse(73)));
      hoodAngleMapHub.put(150d, Rotation2d.fromDegrees(Utils.reverse(72)));
      hoodAngleMapHub.put(160d, Rotation2d.fromDegrees(Utils.reverse(70.5)));
      hoodAngleMapHub.put(170d, Rotation2d.fromDegrees(Utils.reverse(69)));
      hoodAngleMapHub.put(180d, Rotation2d.fromDegrees(Utils.reverse(67.5)));
      hoodAngleMapHub.put(190d, Rotation2d.fromDegrees(Utils.reverse(65.5)));
      hoodAngleMapHub.put(200d, Rotation2d.fromDegrees(Utils.reverse(63.5)));
      hoodAngleMapHub.put(210d, Rotation2d.fromDegrees(Utils.reverse(60.5)));
      hoodAngleMapHub.put(220d, Rotation2d.fromDegrees(Utils.reverse(57)));

      hoodAngleMapNeutral.put(140d, Rotation2d.fromDegrees(Utils.reverse(80.5)));
      hoodAngleMapNeutral.put(150d, Rotation2d.fromDegrees(Utils.reverse(80)));
      hoodAngleMapNeutral.put(160d, Rotation2d.fromDegrees(Utils.reverse(79)));
      hoodAngleMapNeutral.put(170d, Rotation2d.fromDegrees(Utils.reverse(78.5)));
      hoodAngleMapNeutral.put(180d, Rotation2d.fromDegrees(Utils.reverse(77.5)));
      hoodAngleMapNeutral.put(190d, Rotation2d.fromDegrees(Utils.reverse(77)));
      hoodAngleMapNeutral.put(200d, Rotation2d.fromDegrees(Utils.reverse(76.5)));
      hoodAngleMapNeutral.put(210d, Rotation2d.fromDegrees(Utils.reverse(75.5)));
      hoodAngleMapNeutral.put(220d, Rotation2d.fromDegrees(Utils.reverse(75)));
      hoodAngleMapNeutral.put(230d, Rotation2d.fromDegrees(Utils.reverse(74)));
      hoodAngleMapNeutral.put(240d, Rotation2d.fromDegrees(Utils.reverse(73)));
      hoodAngleMapNeutral.put(250d, Rotation2d.fromDegrees(Utils.reverse(72.5)));
      hoodAngleMapNeutral.put(260d, Rotation2d.fromDegrees(Utils.reverse(71.5)));
      hoodAngleMapNeutral.put(270d, Rotation2d.fromDegrees(Utils.reverse(71)));
      hoodAngleMapNeutral.put(280d, Rotation2d.fromDegrees(Utils.reverse(70)));
      hoodAngleMapNeutral.put(290d, Rotation2d.fromDegrees(Utils.reverse(69)));
      hoodAngleMapNeutral.put(300d, Rotation2d.fromDegrees(Utils.reverse(68)));
      hoodAngleMapNeutral.put(310d, Rotation2d.fromDegrees(Utils.reverse(67.5)));
      hoodAngleMapNeutral.put(320d, Rotation2d.fromDegrees(Utils.reverse(66.5)));
      hoodAngleMapNeutral.put(330d, Rotation2d.fromDegrees(Utils.reverse(65.5)));
      hoodAngleMapNeutral.put(340d, Rotation2d.fromDegrees(Utils.reverse(64.5)));
      hoodAngleMapNeutral.put(350d, Rotation2d.fromDegrees(Utils.reverse(63)));
      hoodAngleMapNeutral.put(360d, Rotation2d.fromDegrees(Utils.reverse(62)));
      hoodAngleMapNeutral.put(370d, Rotation2d.fromDegrees(Utils.reverse(61)));
      hoodAngleMapNeutral.put(380d, Rotation2d.fromDegrees(Utils.reverse(59.5)));
      hoodAngleMapNeutral.put(390d, Rotation2d.fromDegrees(Utils.reverse(58.5)));
      hoodAngleMapNeutral.put(400d, Rotation2d.fromDegrees(Utils.reverse(57)));
      hoodAngleMapNeutral.put(410d, Rotation2d.fromDegrees(Utils.reverse(55)));
      hoodAngleMapNeutral.put(420d, Rotation2d.fromDegrees(Utils.reverse(53)));

      hoodAngleMapZone.put(430d, Rotation2d.fromDegrees(Utils.reverse(65)));
      hoodAngleMapZone.put(440d, Rotation2d.fromDegrees(Utils.reverse(64)));
      hoodAngleMapZone.put(450d, Rotation2d.fromDegrees(Utils.reverse(63)));
      hoodAngleMapZone.put(460d, Rotation2d.fromDegrees(Utils.reverse(62.5)));
      hoodAngleMapZone.put(470d, Rotation2d.fromDegrees(Utils.reverse(61.5)));
      hoodAngleMapZone.put(480d, Rotation2d.fromDegrees(Utils.reverse(60.5)));
      hoodAngleMapZone.put(490d, Rotation2d.fromDegrees(Utils.reverse(59.5)));
      hoodAngleMapZone.put(500d, Rotation2d.fromDegrees(Utils.reverse(58.5)));
      hoodAngleMapZone.put(510d, Rotation2d.fromDegrees(Utils.reverse(57.5)));
      hoodAngleMapZone.put(520d, Rotation2d.fromDegrees(Utils.reverse(56)));
      hoodAngleMapZone.put(530d, Rotation2d.fromDegrees(Utils.reverse(54.5)));
      hoodAngleMapZone.put(540d, Rotation2d.fromDegrees(Utils.reverse(53)));
      hoodAngleMapZone.put(550d, Rotation2d.fromDegrees(Utils.reverse(51)));
    }
	}
}