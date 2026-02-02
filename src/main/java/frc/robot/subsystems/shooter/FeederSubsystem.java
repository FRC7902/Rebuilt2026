package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;

import yams.motorcontrollers.remote.TalonFXWrapper;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

/*
	Description:
	Feeder

  Driven by one Kraken x60 motor

	Speed should be match with shooter’s recovery time (time it takes for flywheel to return to correct speed after shooting a fuel)

	Store up to 4 fuel (two per side) in the feeder

	Beam breaks TBD; Current idea to have one beam break at the top, right before the flywheel,
	and two beam breaks on the bottom/entrance of the feeder, one on each side

	Top beam break is about 2” away from the flywheel
 */

public class FeederSubsystem extends SubsystemBase {
	private static final TalonFX feederMotor = new TalonFX(ShooterConstants.FeederID);
	private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
			.withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP)
			.withClosedLoopController(ShooterConstants.FeederKp, 0, 0, DegreesPerSecond.of(360), DegreesPerSecondPerSecond.of(10))
			// Configure Motor and Mechanism properties
			.withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
			.withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
			.withMotorInverted(false)
			// Setup Telemetry
			.withTelemetry("FeederMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
			// Power Optimization
			.withStatorCurrentLimit(Amps.of(120))
			.withClosedLoopRampRate(Seconds.of(0.25))
			.withFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
			.withSimFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
			.withOpenLoopRampRate(Seconds.of(0.25));

	private final SmartMotorController motor = new TalonFXWrapper(feederMotor, DCMotor.getKrakenX60(1), motorConfig);

	private final PivotConfig  pivotConfig = new PivotConfig(motor)
			.withSimColor(new Color8Bit(Color.kOrange))
			.withTelemetry("Feeder", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
			.withMOI(Meters.of(0.25), Kilogram.of(0.1))
			.withHardLimit(Degrees.of(0), Degrees.of(180))
			.withStartingPosition(Degrees.of(90));

	private final Pivot feeder = new Pivot(pivotConfig);

	private static Angle defaultAngle = Degrees.of(180);

	public static Angle getDefaultAngle() {
		return defaultAngle;
	}

	public static void setDefaultVelocity(Angle defaultAngle) {
		FeederSubsystem.defaultAngle = defaultAngle;
	}

	public FeederSubsystem(){

	}
	public Command run() {
		return feeder.setAngle(defaultAngle);
	}
	public Command setAngle(Angle angle) {
		return feeder.setAngle(angle);
	}

	public void setAngleDirect(Angle angle) {
		motor.setPosition(angle);
	}

	public Command setAngle(Supplier<Angle> angleSupplier) {
		return feeder.setAngle(angleSupplier);
	}

	public Angle getAngle() {
		return feeder.getAngle();
	}
	public Command reset(){
		return feeder.setAngle(Degrees.of(0));
	}
	public Command sysId() {
		return feeder.sysId(
				Volts.of(4.0), // maximumVoltage
				Volts.per(Second).of(0.5), // step
				Seconds.of(8.0) // duration
		);
	}

	public Command setDutyCycle(Supplier<Double> dutyCycleSupplier) {
		return feeder.set(dutyCycleSupplier);
	}

	public Command setDutyCycle(double dutyCycle) {
		return feeder.set(dutyCycle);
	}

	@Override
	public void periodic() {
		feeder.updateTelemetry();
	}

	@Override
	public void simulationPeriodic() {
		feeder.simIterate();
	}
}
