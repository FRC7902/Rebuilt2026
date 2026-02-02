package frc.robot;

import static edu.wpi.first.units.Units.Amps;

public final class Constants {
	public static class ShooterConstants {
		//Device ID
		public static final int HoodID = 1;
		public static final int FlywheelID = 1;
		public static final int FeederID = 3;

		public static final double FeederStatorCurrentLimit = 1;
		public static final double FeederSupplyCurrentLimit = 1;

		//Flywheel Constants
		public static final double FlywheelKp = 0.00016541;
		public static final double FlywheelKi = 0;
		public static final double FlywheelKd = 0;

		//hood Constants
		public static final double HoodKp = 0.00016541;
		public static final double HoodKi = 0;
		public static final double HoodKd = 0;

		//Feeder Constants
		public static final double FeederKp = 0.00016541;
		public static final double FeederKi = 0;
		public static final double FeederKd = 0;
		public static final double FeederKs = 0.27937;
		public static final double FeederKv = 0.089836;
		public static final double FeederKa = 0.014557;
	}
}
