package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.*;
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
			.withClosedLoopController(ShooterConstants.FeederKp, 0, 0, DegreesPerSecond.of(360), DegreesPerSecondPerSecond.of(180))
			// Configure Motor and Mechanism properties
			.withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
			.withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
			.withMotorInverted(false)
			// Setup Telemetry
			.withTelemetry("FeederMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
			// Power Optimization
			.withStatorCurrentLimit(Amps.of(40))
			.withClosedLoopRampRate(Seconds.of(0.25))
			.withFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
			.withSimFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
			.withOpenLoopRampRate(Seconds.of(0.25));

	private final SmartMotorController motor = new TalonFXWrapper(feederMotor, DCMotor.getKrakenX60(1), motorConfig);

	private final PivotConfig  pivotConfig = new PivotConfig(motor)
			.withSimColor(new Color8Bit(Color.kOrange))
			.withTelemetry("Feeder", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
			.withMOI(Meters.of(0.20), Kilogram.of(0.1))
			.withHardLimit(Degrees.of(0), Degrees.of(360))
			.withStartingPosition(Degrees.of(0));

	private final Pivot feeder = new Pivot(pivotConfig);

	private static final DigitalInput BeamBreakLeftFeeder = new DigitalInput(ShooterConstants.BeamBreakL);
	private static final DigitalInput BeamBreakRightFeeder = new DigitalInput(ShooterConstants.BeamBreakR);
	private static final DigitalInput BeamBreakTop = new DigitalInput(ShooterConstants.BeamBreakT);

	public static boolean getBeamBreakLeftFeeder(){
		return BeamBreakLeftFeeder.get();
	}
	public static boolean getBeamBreakRightFeeder(){
		return BeamBreakRightFeeder.get();
	}
	public static boolean getBeamBreakTop(){
		return BeamBreakTop.get();
	}

	private static Angle defaultAngle = Degrees.of(180);

	public static Angle getDefaultAngle() {
		return defaultAngle;
	}

	public static void setDefaultAngle(Angle defaultAngle) {
		FeederSubsystem.defaultAngle = defaultAngle;
	}

	public FeederSubsystem(){

	}
	public Command run() {
		return new ConditionalCommand(stop(),
				setAngle(defaultAngle),
				() -> getAngle() == defaultAngle
			);
	}
	public Command setAngle(Angle angle) {
		return new ConditionalCommand(stop(),
				feeder.setAngle(angle),
				() -> getAngle() == angle
		);
	}
	public Command stop(){
		return feeder.setVoltage(Volts.of(0));
	}
	public void setAngleDirect(Angle angle) {
		motor.setPosition(angle);
	}

	public Command setAngle(Supplier<Angle> angleSupplier) {
		return new ConditionalCommand(stop(),
				feeder.setAngle(angleSupplier),
				() -> getAngle() == angleSupplier.get()
		);
	}

	public Angle getAngle() {
		return feeder.getAngle();
	}
	public Command reset(){
		return new ConditionalCommand(stop(),
				setAngle(Degrees.of(0)),
				() -> getAngle() == Degrees.of(0)
		);
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
		SmartDashboard.putBoolean("Feeder/Beam Break left: ", getBeamBreakLeftFeeder());
		SmartDashboard.putBoolean("Feeder/Beam Break right: ", getBeamBreakLeftFeeder());
		SmartDashboard.putBoolean("Feeder/Beam Break top: ", getBeamBreakTop());
		SmartDashboard.putNumber("Feeder/Feeder Angle", getAngle().in(Units.Degrees));
		feeder.updateTelemetry();
	}

	@Override
	public void simulationPeriodic() {
		feeder.simIterate();
	}
}
