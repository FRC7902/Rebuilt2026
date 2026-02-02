package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.DegreesPerSecond;

public class ShooterSubsystem extends SubsystemBase {
	//initialize required subsystems
	private final HoodSubsystem hoodSubsystem = new HoodSubsystem();
	private final FlywheelSubsystem flywheelSubsystem = new FlywheelSubsystem();
	private final FeederSubsystem feederSubsystem = new FeederSubsystem();

	private Supplier<AngularVelocity> flywheelVelocitySupplier = () -> DegreesPerSecond.of(1);

	private PIDController hoodPIDController = new PIDController(
			ShooterConstants.HoodKp, ShooterConstants.HoodKi, ShooterConstants.HoodKd
	);
	private PIDController flywheelPIDController = new PIDController(
			ShooterConstants.FlywheelKp, ShooterConstants.FlywheelKi, ShooterConstants.FlywheelKd
	);


	public ShooterSubsystem() {

	}
	public Command feed(){
		return feederSubsystem.run();
	}
	public Command feed(Angle angle){
		return feederSubsystem.setAngle(angle);
	}
	public Command stopFeeder(){
		return feederSubsystem.reset();
	}
	public Command aimAt(Angle hoodAngle){
		return hoodSubsystem.setAngle(hoodAngle);
	}
	public Command runShooter(){
		if(flywheelVelocitySupplier == null){
			DriverStation.reportWarning("Shooter velocity set to null, not running shooter", true);
			return flywheelSubsystem.idle();
		}
		return flywheelSubsystem.setVelocity(flywheelVelocitySupplier);
	}
	public Command stopShooter(){
		return flywheelSubsystem.setVelocity(DegreesPerSecond.of(0));
	}
	public Command runShooter(AngularVelocity velocity) {
		if (velocity == null) {
			DriverStation.reportWarning("Shooter velocity set to null, defaulting to 0", true);
			velocity = DegreesPerSecond.of(0);
		}

		return flywheelSubsystem.setVelocity(velocity);
	}
	public void setVelocitySupplier(Supplier<AngularVelocity> velocitySupplier) {
		this.flywheelVelocitySupplier = velocitySupplier;
	}

}
