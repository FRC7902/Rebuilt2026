package frc.robot.subsystems.shooter;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FlyWheelConstants;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class FlywheelSubsystem extends SubsystemBase {
	private final TalonFX flywheelMotorLeft = new TalonFX(FlyWheelConstants.FLYWHEEL_LEADER_ID);
	private final TalonFX flywheelMotorRight = new TalonFX(FlyWheelConstants.FLYWHEEL_FOLLOWER_ID);
	private final SmartMotorControllerConfig motorConfig;
	private final SmartMotorController m_motor;
	private final FlyWheelConfig flywheelConfig;
	private final FlyWheel flywheel;

	public Command stop() {
        return this.runOnce(() -> {
			m_motor.stopClosedLoopController();
			m_motor.setDutyCycle(0);
        });
    }

	public FlywheelSubsystem()
	{
		motorConfig = new SmartMotorControllerConfig(this)
			.withClosedLoopController(FlyWheelConstants.FLYWHEEL_KP, FlyWheelConstants.FLYWHEEL_KI, FlyWheelConstants.FLYWHEEL_KD, FlyWheelConstants.FLYWHEEL_MAX_VELOCITY, FlyWheelConstants.FLYWHEEL_MAX_ACCELERATION)
			.withGearing(FlyWheelConstants.FLYWHEEL_GEARING)
			.withIdleMode(FlyWheelConstants.FLYWHEEL_IDLE)
			.withTelemetry("FlywheelMotor", SmartMotorControllerConfig.TelemetryVerbosity.LOW)
			.withStatorCurrentLimit(FlyWheelConstants.FLYWHEEL_STATOR)
			.withMotorInverted(false)
			.withClosedLoopRampRate(FlyWheelConstants.FLYWHEEL_CLOSED_RATE)
			.withOpenLoopRampRate(FlyWheelConstants.FLYWHEEL_OPEN_RATE)
			.withFeedforward(new SimpleMotorFeedforward(FlyWheelConstants.FLYWHEEL_KS, FlyWheelConstants.FLYWHEEL_KV, FlyWheelConstants.FLYWHEEL_KA))
			.withSimFeedforward(new SimpleMotorFeedforward(FlyWheelConstants.FLYWHEEL_KS, FlyWheelConstants.FLYWHEEL_KV, FlyWheelConstants.FLYWHEEL_KA))
			.withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP)
			.withFollowers(Pair.of(flywheelMotorRight, true))
			.withMomentOfInertia(FlyWheelConstants.FLYWHEEL_MOI);
		m_motor = new TalonFXWrapper(flywheelMotorLeft, DCMotor.getKrakenX60Foc(2), motorConfig);
		flywheelConfig = new FlyWheelConfig(m_motor)
			.withDiameter(FlyWheelConstants.FLYWHEEL_DIAMETER)
			.withTelemetry("FlywheelMech", SmartMotorControllerConfig.TelemetryVerbosity.LOW)
			.withSoftLimit(FlyWheelConstants.FLYWHEEL_MAX_VELOCITY.times(-1), FlyWheelConstants.FLYWHEEL_MAX_VELOCITY)
			.withSpeedometerSimulation(FlyWheelConstants.FLYWHEEL_MAX_VELOCITY)
			.withMOI(FlyWheelConstants.FLYWHEEL_MOI);
		flywheel = new FlyWheel(flywheelConfig);
	}

	public AngularVelocity getAngularVelocity()
	{
		return flywheel.getSpeed();
	}

	public Command setAngularVelocity(AngularVelocity speed)
	{
		return flywheel.setSpeed(speed);
	}

	private Command setDutyCycle(double dutyCycle)
	{
		return flywheel.set(dutyCycle);
	}

	public Command setAngularVelocity(Supplier<AngularVelocity> speed)
	{
		return flywheel.setSpeed(speed);
	}

	private Command setDutyCycle(Supplier<Double> dutyCycle)
	{
		return flywheel.set(dutyCycle);
	}

	public Command sysId()
	{
		return flywheel.sysId(Volts.of(10), Volts.of(1).per(Second), Seconds.of(20))
		.beforeStarting(() -> SignalLogger.start()).finallyDo(() -> SignalLogger.stop());
	}

	
	private Command setRPM(LinearVelocity speed)
	{
		return flywheel.setSpeed(RotationsPerSecond.of(speed.in(MetersPerSecond) / FlyWheelConstants.FLYWHEEL_DIAMETER.times(Math.PI).in(Meters)));
	}

	private void setRPMDirect(LinearVelocity speed)
	{
		m_motor.setVelocity(RotationsPerSecond.of(speed.in(MetersPerSecond) / FlyWheelConstants.FLYWHEEL_DIAMETER.times(Math.PI).in(Meters)));
	}

	
}