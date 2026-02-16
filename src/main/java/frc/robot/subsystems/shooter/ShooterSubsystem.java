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
		hoodAngleMap.put(35d, Rotation2d.fromDegrees(87));
		hoodAngleMap.put(45d, Rotation2d.fromDegrees(86));
		hoodAngleMap.put(55d, Rotation2d.fromDegrees(85));
		hoodAngleMap.put(65d, Rotation2d.fromDegrees(84));
		hoodAngleMap.put(75d, Rotation2d.fromDegrees(83));
		hoodAngleMap.put(85d, Rotation2d.fromDegrees(82));
		hoodAngleMap.put(95d, Rotation2d.fromDegrees(81));
		hoodAngleMap.put(105d, Rotation2d.fromDegrees(80));
		hoodAngleMap.put(115d, Rotation2d.fromDegrees(79));
		hoodAngleMap.put(125d, Rotation2d.fromDegrees(78));
		hoodAngleMap.put(135d, Rotation2d.fromDegrees(77));
		hoodAngleMap.put(145d, Rotation2d.fromDegrees(76));
		hoodAngleMap.put(155d, Rotation2d.fromDegrees(75));
		hoodAngleMap.put(165d, Rotation2d.fromDegrees(73.5));
		hoodAngleMap.put(175d, Rotation2d.fromDegrees(72.5));
		hoodAngleMap.put(185d, Rotation2d.fromDegrees(71.5));
		hoodAngleMap.put(195d, Rotation2d.fromDegrees(70));
		hoodAngleMap.put(205d, Rotation2d.fromDegrees(69));
		hoodAngleMap.put(215d, Rotation2d.fromDegrees(67.5));
		hoodAngleMap.put(225d, Rotation2d.fromDegrees(65.5));
		hoodAngleMap.put(235d, Rotation2d.fromDegrees(64));
		hoodAngleMap.put(245d, Rotation2d.fromDegrees(62));
		hoodAngleMap.put(255d, Rotation2d.fromDegrees(60));
		hoodAngleMap.put(265d, Rotation2d.fromDegrees(57));
		hoodAngleMap.put(275d, Rotation2d.fromDegrees(54));
		hoodAngleMap.put(280d, Rotation2d.fromDegrees(50));
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
