package frc.robot;

import edu.wpi.first.units.measure.*;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorControllerConfig;

import static edu.wpi.first.units.Units.*;

public final class Constants {
	public static class ShooterConstants {

		//Flywheel
		public static final int FLYWHEEL_ID = 1;
		//TODO: needs to be tuned
		public static final double FLYWHEEL_KP = 1;
		public static final double FLYWHEEL_KI = 1;
		public static final double FLYWHEEL_KD = 1;
		public static final AngularVelocity FLYWHEEL_MAX_VELOCITY = RPM.of(5000);
		public static final AngularAcceleration FLYWHEEL_MAX_ACCELERATION = RotationsPerSecondPerSecond.of(2500);
		public static final MechanismGearing FLYWHEEL_GEARING = new MechanismGearing(GearBox.fromReductionStages(3, 4));
		public static final SmartMotorControllerConfig.MotorMode FLYWHEEL_IDLE = SmartMotorControllerConfig.MotorMode.COAST;
		public static final Current FLYWHEEL_STATOR = Amps.of(50);
		public static final Time FLYWHEEL_CLOSED_RATE = Seconds.of(1);
		public static final Time FLYWHEEL_OPEN_RATE = Seconds.of(1);
		public static final double FLYWHEEL_KS = 1;
		public static final double FLYWHEEL_KV = 1;
		public static final double FLYWHEEL_KA = 1;

		public static double FLYWHEEL_VELOCITY_SUPPLIER = 1;
		public static final Distance FLYWHEEL_DIAMETER = Inches.of(2.943);
		public static final Mass FLYWHEEL_MASS = Pounds.of(0.14);
		public static final AngularVelocity FLYWHEEL_LIMIT_LOW = RPM.of(6500);
		public static final AngularVelocity FLYWHEEL_LIMIT_HIGH = RPM.of(11568);

		//Hood
		public static final int HOOD_ID = 2;
		//TODO: needs to be tuned
		public static final double HOOD_KP = 5;
		public static final double HOOD_KI = 1;
		public static final double HOOD_KD = 1;
		public static final AngularVelocity HOOD_MAX_VELOCITY = RPM.of(5000);
		public static final AngularAcceleration HOOD_MAX_ACCELERATION = RotationsPerSecondPerSecond.of(2500);
		public static final MechanismGearing HOOD_GEARING = new MechanismGearing(GearBox.fromReductionStages(3, 4));
		public static final SmartMotorControllerConfig.MotorMode HOOD_MOTOR_IDLE_MODE = SmartMotorControllerConfig.MotorMode.COAST;
		public static final Current HOOD_STATOR = Amps.of(40);
		public static final Time HOOD_CLOSED_RATE = Seconds.of(0.25);
		public static final Time HOOD_OPEN_RATE = Seconds.of(0.25);
		public static final double HOOD_KS = 1;
		public static final double HOOD_KV = 1;
		public static final double HOOD_KA = 1;

		public static final Distance HOOD_LENGTH = Inches.of(8.884333);
		public static final MomentOfInertia HOOD_MOI = KilogramSquareMeters.of(0.0664243573);
		public static final Angle HOOD_SOFT_LIMIT_LOW = Degrees.of(44);
		public static final Angle HOOD_SOFT_LIMIT_HIGH = Degrees.of(87);
		public static final Angle HOOD_HARD_LIMIT_LOW = Degrees.of(0);
		public static final Angle  HOOD_HARD_LIMIT_HIGH = Degrees.of(90);
		public static final Angle HOOD_START_POSITION = Degrees.of(72);

		//Feeder
		public static final int FEEDER_ID = 3;
		public static final double FEEDER_KP = 10;
		public static final double FEEDER_KI = 1;
		public static final double FEEDER_KD = 1;
		public static final double FEEDER_STATOR = 480;
		public static final double FEEDER_SUPPLY = 480;
		public static final double FEEDER_KV = 10;
		public static final double FEEDER_KA = 8;

		public static final double FEEDER_SIM_LENGTH = 0.0762;

		public static final double FEEDER_TIME_PERIOD = 2;
		public static final double FEEDER_GEAR_RATIO = 20;
		public static final double FEEDER_VOLTAGE = 12;

		public static final int BEAM_BREAK_LEFT_ID = 4;
		public static final int BEAM_BREAK_RIGHT_ID = 5;
		public static final int BEAM_BREAK_TOP_ID = 6;

	}
}
