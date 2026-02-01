package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.remote.TalonFXWrapper;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class HoodSubsystem extends SubsystemBase {
	private final TalonFX hoodMotor = new TalonFX(ShooterConstants.HoodID);
	private TalonFXSimState  hoodSim = hoodMotor.getSimState();

	private final SmartMotorControllerConfig hoodMotorConfig = new SmartMotorControllerConfig(this)
			.withClosedLoopController(0.00016541, 0, 0, RPM.of(5000), RotationsPerSecondPerSecond.of(2500))
			.withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
			.withIdleMode(SmartMotorControllerConfig.MotorMode.COAST)
			.withTelemetry("HoodMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
			.withStatorCurrentLimit(Amps.of(40))
			.withMotorInverted(false)
			.withClosedLoopRampRate(Seconds.of(0.25))
			.withOpenLoopRampRate(Seconds.of(0.25))
			.withFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
			.withSimFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
			.withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP);

	private final SmartMotorController hoodSMC = new TalonFXWrapper(hoodMotor, DCMotor.getKrakenX44(1), hoodMotorConfig);
	MomentOfInertia Inertia = KilogramSquareMeters.of(0.25);
	private final ArmConfig hoodConfig = new ArmConfig(hoodSMC)
			.withStartingPosition(Degrees.of(0))
			.withLength(Inches.of(3))
			.withMOI(Inertia)
			.withTelemetry("HoodMech", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
			.withSoftLimits(Degrees.of(5), Degrees.of(100))
			.withHardLimit(Degrees.of(0), Degrees.of(120)); // The Hood can be modeled as an arm since it has a
	// gravitational force acted upon based on the angle its in

	private final Arm hood = new Arm(hoodConfig);

	public HoodSubsystem() {

	}

	public Command setAngle(Angle angle) {
		return hood.setAngle(angle);
	}

	public void setAngleDirect(Angle angle)
	{
		hoodSMC.setPosition(angle);
	}

	public Command setAngle(Supplier<Angle> angleSupplier) {
		return hood.setAngle(angleSupplier);
	}

	public Angle getAngle() {
		return hood.getAngle();
	}

	public Command sysId() {
		return hood.sysId(
				Volts.of(4.0), // maximumVoltage
				Volts.per(Second).of(0.5), // step
				Seconds.of(8.0) // duration
		);
	}

	public Command setDutyCycle(Supplier<Double> dutyCycleSupplier) {
		return hood.set(dutyCycleSupplier);
	}

	public Command setDutyCycle(double dutyCycle) {
		return hood.set(dutyCycle);
	}

	@Override
	public void periodic() {
		hood.updateTelemetry();
	}

	@Override
	public void simulationPeriodic() {
		hood.simIterate();
	}
}
