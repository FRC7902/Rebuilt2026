package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

public class ShooterSubsystem extends SubsystemBase {
	//initialize required subsystems
	private final HoodSubsystem hoodSubsystem;
	private final FlywheelSubsystem flywheelSubsystem;
	private final FeederSubsystem feederSubsystem;

	//Necessary variables
	private Supplier<AngularVelocity> flywheelVelocitySupplier = () -> DegreesPerSecond.of(1);
	private static Angle defaultAngle = Degrees.of(90);

//empty constructor
	public ShooterSubsystem() {
		hoodSubsystem = new HoodSubsystem();
		flywheelSubsystem = new FlywheelSubsystem();
		feederSubsystem = new FeederSubsystem();
	}
	//FeederSubsystem Commands
	public Command feed(){
		return feederSubsystem.run();
	}
	public Command feed(Angle angle){
		return feederSubsystem.setAngle(angle);
	}
	public Command reset(){
		return feederSubsystem.reset();
	}
	public Command stopFeeder(){
		return feederSubsystem.stop();
	}

	//Hood aiming
	public Command aimAt(Angle hoodAngle){
		return hoodSubsystem.setAngle(hoodAngle);
	}

	//Shooter Commands
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
	private static boolean shootContinuous = false;
	public static void switchContinuous(){
		shootContinuous = !shootContinuous;
	}
	//Feeder Beam Break usage
	public Command runFeeder(){
		return new ParallelCommandGroup(
				feederSubsystem.run().repeatedly(),
				new ConditionalCommand(
						new SequentialCommandGroup(
								aimAt(defaultAngle),
								runShooter(),
								new ConditionalCommand(
										new InstantCommand(),
										new SequentialCommandGroup(
												new WaitCommand(2),
												stopShooter()
										),
										() -> shootContinuous
								)
						),
						new InstantCommand(),
						FeederSubsystem::getBeamBreakTop
				)
		);
	}
	public Command runContinuous(){
		return new ConditionalCommand(
				new InstantCommand(),
				new ParallelCommandGroup(
						new InstantCommand(FeederSubsystem::StartTimer),
						runFeeder()
				),
				FeederSubsystem::isTimerEnded
		);

	}
	@Override
	public void periodic() {
		if(FeederSubsystem.isTimerEnded()){
			stopFeeder();
			stopShooter();
		}
	}
	public void setDefaultAngle(Angle angle){
		defaultAngle = angle;
	}
}
