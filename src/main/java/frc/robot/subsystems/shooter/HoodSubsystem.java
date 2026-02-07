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
	private final TalonFX hoodMotor = new TalonFX(ShooterConstants.HOOD_ID);
	private TalonFXSimState  hoodSim = hoodMotor.getSimState();

	private final SmartMotorControllerConfig hoodMotorConfig = new SmartMotorControllerConfig(this)
			.withClosedLoopController(ShooterConstants.HOOD_KP, ShooterConstants.HOOD_KI, ShooterConstants.HOOD_KD, ShooterConstants.HOOD_MAX_VELOCITY, ShooterConstants.HOOD_MAX_ACCELERATION)
			.withGearing(ShooterConstants.HOOD_GEARING)
			.withIdleMode(ShooterConstants.HOOD_IDLE)
			.withTelemetry("HoodMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
			.withStatorCurrentLimit(ShooterConstants.HOOD_STATOR)
			.withMotorInverted(false)
			.withClosedLoopRampRate(ShooterConstants.HOOD_CLOSED_RATE)
			.withOpenLoopRampRate(ShooterConstants.HOOD_OPEN_RATE)
			.withFeedforward(new SimpleMotorFeedforward(ShooterConstants.HOOD_KS, ShooterConstants.HOOD_KV, ShooterConstants.HOOD_KA))
			.withSimFeedforward(new SimpleMotorFeedforward(ShooterConstants.HOOD_KS, ShooterConstants.HOOD_KV, ShooterConstants.HOOD_KA))
			.withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP);

	private final SmartMotorController hoodSMC = new TalonFXWrapper(hoodMotor, DCMotor.getKrakenX44(1), hoodMotorConfig);
	private final ArmConfig hoodConfig = new ArmConfig(hoodSMC)
			.withStartingPosition(ShooterConstants.HOOD_START_POSITION)
			.withLength(ShooterConstants.FEEDER_LENGTH)
			.withMOI(ShooterConstants.HOOD_MOI)
			.withTelemetry("HoodMech", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
			.withSoftLimits(ShooterConstants.HOOD_SOFT_LIMIT_LOW, ShooterConstants.HOOD_SOFT_LIMIT_HIGH)
			.withHardLimit(ShooterConstants.HOOD_HARD_LIMIT_LOW, ShooterConstants.HOOD_HARD_LIMIT_HIGH);

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
