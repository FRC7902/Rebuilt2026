package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterConstants;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.remote.TalonFXWrapper;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class FlywheelSubsystem extends SubsystemBase {
	private final TalonFX flywheelMotor = new TalonFX(ShooterConstants.FLYWHEEL_ID);

	private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
			.withClosedLoopController(ShooterConstants.FLYWHEEL_KP, ShooterConstants.FLYWHEEL_KI, ShooterConstants.FLYWHEEL_KD, ShooterConstants.FLYWHEEL_MAX_VELOCITY, ShooterConstants.FLYWHEEL_MAX_ACCELERATION)
			.withGearing(ShooterConstants.FLYWHEEL_GEARING)
			.withIdleMode(ShooterConstants.FLYWHEEL_IDLE)
			.withTelemetry("FlywheelMotor", SmartMotorControllerConfig.TelemetryVerbosity.LOW)
			.withStatorCurrentLimit(ShooterConstants.FLYWHEEL_STATOR)
			.withMotorInverted(false)
			.withClosedLoopRampRate(ShooterConstants.FLYWHEEL_CLOSED_RATE)
			.withOpenLoopRampRate(ShooterConstants.FLYWHEEL_OPEN_RATE)
			.withFeedforward(new SimpleMotorFeedforward(ShooterConstants.FLYWHEEL_KS, ShooterConstants.FLYWHEEL_KV, ShooterConstants.FLYWHEEL_KA))
			.withSimFeedforward(new SimpleMotorFeedforward(ShooterConstants.FLYWHEEL_KS, ShooterConstants.FLYWHEEL_KV, ShooterConstants.FLYWHEEL_KA))
			.withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP);

	private final SmartMotorController motor = new TalonFXWrapper(flywheelMotor, DCMotor.getKrakenX60Foc(2), motorConfig);

	private final FlyWheelConfig flywheelConfig = new FlyWheelConfig(motor)
			.withDiameter(ShooterConstants.FLYWHEEL_DIAMETER)
			.withMass(ShooterConstants.FLYWHEEL_MASS)
			.withTelemetry("FlywheelMech", SmartMotorControllerConfig.TelemetryVerbosity.LOW)
			.withSoftLimit(ShooterConstants.FLYWHEEL_LIMIT_LOW, ShooterConstants.FLYWHEEL_LIMIT_HIGH)
			.withSpeedometerSimulation(ShooterConstants.FLYWHEEL_MAX_VELOCITY);

	private final FlyWheel flywheel = new FlyWheel(flywheelConfig);

	public FlywheelSubsystem()
	{
	}

	public AngularVelocity getVelocity()
	{
		return flywheel.getSpeed();
	}

	public Command setVelocity(AngularVelocity speed)
	{
		return flywheel.setSpeed(speed);
	}

	public Command setDutyCycle(double dutyCycle)
	{
		return flywheel.set(dutyCycle);
	}

	public Command setVelocity(Supplier<AngularVelocity> speed)
	{
		return flywheel.setSpeed(speed);
	}

	public Command setDutyCycle(Supplier<Double> dutyCycle)
	{
		return flywheel.set(dutyCycle);
	}

	public Command sysId()
	{
		return flywheel.sysId(Volts.of(10), Volts.of(1).per(Second), Seconds.of(5));
	}

	@Override
	public void periodic()
	{
		flywheel.updateTelemetry();
	}

	@Override
	public void simulationPeriodic()
	{
		flywheel.simIterate();
	}

	public Command setRPM(LinearVelocity speed)
	{
		return flywheel.setSpeed(RotationsPerSecond.of(speed.in(MetersPerSecond) / ShooterConstants.FLYWHEEL_DIAMETER.times(Math.PI).in(Meters)));
	}

	public void setRPMDirect(LinearVelocity speed)
	{
		motor.setVelocity(RotationsPerSecond.of(speed.in(MetersPerSecond) / ShooterConstants.FLYWHEEL_DIAMETER.times(Math.PI).in(Meters)));
	}
}
