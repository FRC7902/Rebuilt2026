package frc.robot.subsystems.shooter;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class HoodSubsystem extends SubsystemBase {
	private final TalonFX m_hoodMotor = new TalonFX(HoodConstants.HOOD_ID);

	private final SmartMotorControllerConfig m_hoodMotorConfig;
	private final SmartMotorController m_hoodSMC;
	private final ArmConfig m_hoodConfig;
	private final Arm m_hood;

	public HoodSubsystem() {
		m_hoodMotorConfig = new SmartMotorControllerConfig(this)
			.withClosedLoopController(HoodConstants.HOOD_KP, HoodConstants.HOOD_KI, HoodConstants.HOOD_KD, HoodConstants.HOOD_MAX_VELOCITY, HoodConstants.HOOD_MAX_ACCELERATION)
			.withSimClosedLoopController(HoodConstants.HOOD_SIM_KP, HoodConstants.HOOD_SIM_KI, HoodConstants.HOOD_SIM_KD, HoodConstants.HOOD_MAX_VELOCITY, HoodConstants.HOOD_MAX_ACCELERATION)
			.withGearing(HoodConstants.HOOD_GEARING)
			.withIdleMode(HoodConstants.HOOD_MOTOR_IDLE_MODE)
			.withTelemetry("HoodMotor", SmartMotorControllerConfig.TelemetryVerbosity.LOW)
			.withStatorCurrentLimit(HoodConstants.HOOD_STATOR)
			.withSupplyCurrentLimit(HoodConstants.HOOD_SUPPLY)
			.withMotorInverted(false)
			.withClosedLoopRampRate(HoodConstants.HOOD_CLOSED_RATE)
			.withOpenLoopRampRate(HoodConstants.HOOD_OPEN_RATE)
			.withFeedforward(new SimpleMotorFeedforward(HoodConstants.HOOD_KS, HoodConstants.HOOD_KV, HoodConstants.HOOD_KA))
			.withSimFeedforward(new SimpleMotorFeedforward(HoodConstants.HOOD_KS, HoodConstants.HOOD_KV, HoodConstants.HOOD_KA))
			.withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP);
		m_hoodSMC = new TalonFXWrapper(m_hoodMotor, DCMotor.getKrakenX44Foc(1), m_hoodMotorConfig);
		m_hoodConfig = new ArmConfig(m_hoodSMC)
			.withStartingPosition(HoodConstants.HOOD_SOFT_LIMIT_LOW)
			.withLength(HoodConstants.HOOD_LENGTH)
			.withMOI(HoodConstants.HOOD_MOI)
			.withTelemetry("HoodMech", SmartMotorControllerConfig.TelemetryVerbosity.LOW)
			.withSoftLimits(HoodConstants.HOOD_SOFT_LIMIT_LOW, HoodConstants.HOOD_SOFT_LIMIT_HIGH)
			.withHardLimit(HoodConstants.HOOD_HARD_LIMIT_LOW, HoodConstants.HOOD_HARD_LIMIT_HIGH);
		m_hood = new Arm(m_hoodConfig);
	}

	public Command setAngle(Angle angle) {
		return m_hood.setAngle(angle);
	}

	public void setAngleDirect(Angle angle)
	{
		m_hoodSMC.setPosition(angle);
	}

	public Command setAngle(Supplier<Angle> angleSupplier) {
		return m_hood.setAngle(angleSupplier);
	}

	public Angle getAngle() {
		return m_hood.getAngle();
	}

	private Command sysId() {
		return m_hood.sysId(
				Volts.of(4.0), // maximumVoltage
				Volts.per(Second).of(0.5), // step
				Seconds.of(8.0) // duration
		);
	}

	private Command setDutyCycle(Supplier<Double> dutyCycleSupplier) {
		return m_hood.set(dutyCycleSupplier);
	}

	private Command setDutyCycle(double dutyCycle) {
		return m_hood.set(dutyCycle);
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Hood degrees", getAngle().in(Degrees));
		m_hood.updateTelemetry();
	}

	@Override
	public void simulationPeriodic() {
		m_hood.simIterate();
	}
}