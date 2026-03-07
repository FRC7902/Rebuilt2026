package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
	//initialise required subsystems
	private final HoodSubsystem m_hoodSubsystem;
	private final FlywheelSubsystem m_flywheelSubsystem;
	private final FeederSubsystem m_feederSubsystem;

	//Necessary variables

	public ShooterSubsystem() {
		m_hoodSubsystem = new HoodSubsystem();
		m_flywheelSubsystem = new FlywheelSubsystem();
		m_feederSubsystem = new FeederSubsystem();
	}

	public void runFeeder(AngularVelocity speed){
		m_feederSubsystem.setVelocity(speed);
	}
	public void stopFeeder(){
		m_feederSubsystem.stop();
	}
	public Command stopFlyWheel(){
		return m_flywheelSubsystem.stop();
	}
	public Angle getHoodAngle(){
		return m_hoodSubsystem.getAngle();
	}
	//Hood aiming
	public Command aimAtDistanceHub(double distanceInches){
		return m_hoodSubsystem.setAngle(ShooterConstants.hoodAngleMapHub.get(distanceInches).getMeasure());
	}
	public Command aimAtDistanceNeutral(double distanceInches){
		return m_hoodSubsystem.setAngle(ShooterConstants.hoodAngleMapNeutral.get(distanceInches).getMeasure());
	}
	public Command aimAtDistanceZone(double distanceInches){
		return m_hoodSubsystem.setAngle(ShooterConstants.hoodAngleMapZone.get(distanceInches).getMeasure());
	}
	public Command aimAt(Angle angle){
		return m_hoodSubsystem.setAngle(angle);
	}

	//Shooter Commands
	public Command stopShooter(){
		return m_flywheelSubsystem.setAngularVelocity(DegreesPerSecond.of(0));
	}
	public Command runShooter(AngularVelocity velocity) {
		if (velocity == null) {
			DriverStation.reportWarning("Shooter velocity set to null, defaulting to 0", true);
			velocity = DegreesPerSecond.of(0);
		}
		return m_flywheelSubsystem.setAngularVelocity(velocity);
	}
	public AngularVelocity getAngularVelocity(){
		return m_flywheelSubsystem.getAngularVelocity();
	}
	public Command sysIdFlywheel(){
		return m_flywheelSubsystem.sysId();
	}
}