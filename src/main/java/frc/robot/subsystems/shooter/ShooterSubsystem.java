package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;

import frc.robot.Constants.ShooterConstants;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

public class ShooterSubsystem extends SubsystemBase {
	//initialise required subsystems
	private final HoodSubsystem hoodSubsystem;
	private final FlywheelSubsystem flywheelSubsystem;
	private final FeederSubsystem feederSubsystem;

	private static final InterpolatingTreeMap<Double, Rotation2d> hoodAngleMap =
			new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);

	static{
		hoodAngleMap.put(35d, Rotation2d.fromDegrees(85));
		hoodAngleMap.put(37d, Rotation2d.fromDegrees(84));
		hoodAngleMap.put(39d, Rotation2d.fromDegrees(83.5));
		hoodAngleMap.put(41d, Rotation2d.fromDegrees(82.5));
		hoodAngleMap.put(43d, Rotation2d.fromDegrees(81));
		hoodAngleMap.put(45d, Rotation2d.fromDegrees(80));
		hoodAngleMap.put(47d, Rotation2d.fromDegrees(79));
		hoodAngleMap.put(49d, Rotation2d.fromDegrees(78));
		hoodAngleMap.put(51d, Rotation2d.fromDegrees(77));
		hoodAngleMap.put(53d, Rotation2d.fromDegrees(75.5));
		hoodAngleMap.put(55d, Rotation2d.fromDegrees(75.5));
		hoodAngleMap.put(57d, Rotation2d.fromDegrees(77.3));
		hoodAngleMap.put(76d, Rotation2d.fromDegrees(72));
		hoodAngleMap.put(90d, Rotation2d.fromDegrees(66));
		hoodAngleMap.put(60d, Rotation2d.fromDegrees(74.5));
		hoodAngleMap.put(63d, Rotation2d.fromDegrees(73.5));
		hoodAngleMap.put(66d, Rotation2d.fromDegrees(72.5));
		hoodAngleMap.put(69d, Rotation2d.fromDegrees(71.5));
		hoodAngleMap.put(72d, Rotation2d.fromDegrees(70));
		hoodAngleMap.put(75d, Rotation2d.fromDegrees(69));
		hoodAngleMap.put(78d, Rotation2d.fromDegrees(67.5));
		hoodAngleMap.put(81d, Rotation2d.fromDegrees(66.5));
		hoodAngleMap.put(84d, Rotation2d.fromDegrees(64.5));
		hoodAngleMap.put(87d, Rotation2d.fromDegrees(62.5));
	}

	private static boolean shootContinuous = false;

	//Necessary variables
	private Supplier<AngularVelocity> flywheelVelocitySupplier = () -> DegreesPerSecond.of(ShooterConstants.FLYWHEEL_VELOCITY_SUPPLIER);
	private static Angle defaultAngle = Degrees.of(90);

	public ShooterSubsystem() {
		hoodSubsystem = new HoodSubsystem();
		flywheelSubsystem = new FlywheelSubsystem();
		feederSubsystem = new FeederSubsystem();
	}

	public void runFeeder(){
		feederSubsystem.run();
	}
	public void stopFeeder(){
		feederSubsystem.stop();
	}


	//Hood aiming
	public Command aimAt(double distanceInches){
		return hoodSubsystem.setAngle(hoodAngleMap.get(distanceInches).getMeasure());
	}

	//Shooter Commands
	public Command runShooter(){
		if(flywheelVelocitySupplier == null){
			DriverStation.reportWarning("Shooter velocity set to null, not running shooter", true);
			return flywheelSubsystem.idle();
		}
		return flywheelSubsystem.setAngularVelocity(flywheelVelocitySupplier);
	}
	public Command stopShooter(){
		return flywheelSubsystem.setAngularVelocity(DegreesPerSecond.of(0));
	}
	public Command runShooter(AngularVelocity velocity) {
		if (velocity == null) {
			DriverStation.reportWarning("Shooter velocity set to null, defaulting to 0", true);
			velocity = DegreesPerSecond.of(0);
		}

		return flywheelSubsystem.setAngularVelocity(velocity);
	}
	public void setVelocitySupplier(Supplier<AngularVelocity> velocitySupplier) {
		this.flywheelVelocitySupplier = velocitySupplier;
	}

	public static void toggleContinuousShooting(){
		shootContinuous = !shootContinuous;
	}

	public static boolean isHopperAlmostEmpty(){
		return FeederSubsystem.isHopperAlmostEmpty();
	}
	public void setDefaultAngle(Angle angle){
		defaultAngle = angle;
	}
}
