package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
	//initialise required subsystems
	private final HoodSubsystem hoodSubsystem;
	private final FlywheelSubsystem flywheelSubsystem;
	private final FeederSubsystem feederSubsystem;

	private static final InterpolatingTreeMap<Double, Rotation2d> hoodAngleMapHub =
			new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
	private static final InterpolatingTreeMap<Double, Rotation2d> hoodAngleMapNeutral =
			new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
	private static final InterpolatingTreeMap<Double, Rotation2d> hoodAngleMapZone =
			new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);

	static{
		hoodAngleMapHub.put(36d, Rotation2d.fromDegrees(reverse(84.5)));
		hoodAngleMapHub.put(40d, Rotation2d.fromDegrees(reverse(84.5)));
		hoodAngleMapHub.put(50d, Rotation2d.fromDegrees(reverse(84.5)));
		hoodAngleMapHub.put(60d, Rotation2d.fromDegrees(reverse(83.5)));
		hoodAngleMapHub.put(70d, Rotation2d.fromDegrees(reverse(82.5)));
		hoodAngleMapHub.put(80d, Rotation2d.fromDegrees(reverse(81.5)));
		hoodAngleMapHub.put(90d, Rotation2d.fromDegrees(reverse(80)));
		hoodAngleMapHub.put(100d, Rotation2d.fromDegrees(reverse(79)));
		hoodAngleMapHub.put(110d, Rotation2d.fromDegrees(reverse(77.5)));
		hoodAngleMapHub.put(120d, Rotation2d.fromDegrees(reverse(76)));
		hoodAngleMapHub.put(130d, Rotation2d.fromDegrees(reverse(74.5)));
		hoodAngleMapHub.put(140d, Rotation2d.fromDegrees(reverse(73)));
		hoodAngleMapHub.put(150d, Rotation2d.fromDegrees(reverse(72)));
		hoodAngleMapHub.put(160d, Rotation2d.fromDegrees(reverse(70.5)));
		hoodAngleMapHub.put(170d, Rotation2d.fromDegrees(reverse(69)));
		hoodAngleMapHub.put(180d, Rotation2d.fromDegrees(reverse(67.5)));
		hoodAngleMapHub.put(190d, Rotation2d.fromDegrees(reverse(65.5)));
		hoodAngleMapHub.put(200d, Rotation2d.fromDegrees(reverse(63.5)));
		hoodAngleMapHub.put(210d, Rotation2d.fromDegrees(reverse(60.5)));
		hoodAngleMapHub.put(220d, Rotation2d.fromDegrees(reverse(57)));

		hoodAngleMapNeutral.put(140d, Rotation2d.fromDegrees(reverse(80.5)));
		hoodAngleMapNeutral.put(150d, Rotation2d.fromDegrees(reverse(80)));
		hoodAngleMapNeutral.put(160d, Rotation2d.fromDegrees(reverse(79)));
		hoodAngleMapNeutral.put(170d, Rotation2d.fromDegrees(reverse(78.5)));
		hoodAngleMapNeutral.put(180d, Rotation2d.fromDegrees(reverse(77.5)));
		hoodAngleMapNeutral.put(190d, Rotation2d.fromDegrees(reverse(77)));
		hoodAngleMapNeutral.put(200d, Rotation2d.fromDegrees(reverse(76.5)));
		hoodAngleMapNeutral.put(210d, Rotation2d.fromDegrees(reverse(75.5)));
		hoodAngleMapNeutral.put(220d, Rotation2d.fromDegrees(reverse(75)));
		hoodAngleMapNeutral.put(230d, Rotation2d.fromDegrees(reverse(74)));
		hoodAngleMapNeutral.put(240d, Rotation2d.fromDegrees(reverse(73)));
		hoodAngleMapNeutral.put(250d, Rotation2d.fromDegrees(reverse(72.5)));
		hoodAngleMapNeutral.put(260d, Rotation2d.fromDegrees(reverse(71.5)));
		hoodAngleMapNeutral.put(270d, Rotation2d.fromDegrees(reverse(71)));
		hoodAngleMapNeutral.put(280d, Rotation2d.fromDegrees(reverse(70)));
		hoodAngleMapNeutral.put(290d, Rotation2d.fromDegrees(reverse(69)));
		hoodAngleMapNeutral.put(300d, Rotation2d.fromDegrees(reverse(68)));
		hoodAngleMapNeutral.put(310d, Rotation2d.fromDegrees(reverse(67.5)));
		hoodAngleMapNeutral.put(320d, Rotation2d.fromDegrees(reverse(66.5)));
		hoodAngleMapNeutral.put(330d, Rotation2d.fromDegrees(reverse(65.5)));
		hoodAngleMapNeutral.put(340d, Rotation2d.fromDegrees(reverse(64.5)));
		hoodAngleMapNeutral.put(350d, Rotation2d.fromDegrees(reverse(63)));
		hoodAngleMapNeutral.put(360d, Rotation2d.fromDegrees(reverse(62)));
		hoodAngleMapNeutral.put(370d, Rotation2d.fromDegrees(reverse(61)));
		hoodAngleMapNeutral.put(380d, Rotation2d.fromDegrees(reverse(59.5)));
		hoodAngleMapNeutral.put(390d, Rotation2d.fromDegrees(reverse(58.5)));
		hoodAngleMapNeutral.put(400d, Rotation2d.fromDegrees(reverse(57)));
		hoodAngleMapNeutral.put(410d, Rotation2d.fromDegrees(reverse(55)));
		hoodAngleMapNeutral.put(420d, Rotation2d.fromDegrees(reverse(53)));

		hoodAngleMapZone.put(500d, Rotation2d.fromDegrees(reverse(50)));
	}

	public static double reverse(double angle){
		return Math.abs(90 - angle);
	}

	private static boolean shootContinuous = false;

	//Necessary variables

	public ShooterSubsystem() {
		hoodSubsystem = new HoodSubsystem();
		flywheelSubsystem = new FlywheelSubsystem();
		feederSubsystem = new FeederSubsystem();
	}

	public void runFeeder(double speed){
		feederSubsystem.run(speed);
	}
	public void stopFeeder(){
		feederSubsystem.stop();
	}
	public Command stopFlyWheel(){
		return flywheelSubsystem.stop();
	}
	public Angle getHoodAngle(){
		return hoodSubsystem.getAngle();
	}
	//Hood aiming
	public Command aimAtDistanceHub(double distanceInches){
		return hoodSubsystem.setAngle(hoodAngleMapHub.get(distanceInches).getMeasure());
	}
	public Command aimAtDistanceNeutral(double distanceInches){
		return hoodSubsystem.setAngle(hoodAngleMapNeutral.get(distanceInches).getMeasure());
	}
	public Command aimAtDistanceZone(double distanceInches){
		return hoodSubsystem.setAngle(hoodAngleMapZone.get(distanceInches).getMeasure());
	}
	public Command aimAt(Angle angle){
		return hoodSubsystem.setAngle(angle);
	}

	//Shooter Commands
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
	public AngularVelocity getAngularVelocity(){
		return flywheelSubsystem.getAngularVelocity();
	}

	public static void toggleContinuousShooting(){
		shootContinuous = !shootContinuous;
	}

	public static boolean isHopperAlmostEmpty(){
		return FeederSubsystem.isHopperAlmostEmpty();
	}
	public Command sysIdFlywheel(){
		return flywheelSubsystem.sysId();
	}
}
