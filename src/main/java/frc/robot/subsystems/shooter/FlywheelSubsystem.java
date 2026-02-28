package frc.robot.subsystems.shooter;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class FlywheelSubsystem extends SubsystemBase {
	private final TalonFX flywheelMotorLeft = new TalonFX(ShooterConstants.FLYWHEEL_LEADER_ID);
	private final TalonFX flywheelMotorRight = new TalonFX(ShooterConstants.FLYWHEEL_FOLLOWER_ID);

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
			.withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP)
			.withFollowers(Pair.of(flywheelMotorRight, true))
			.withMomentOfInertia(ShooterConstants.FLYWHEEL_MOI);

	private final SmartMotorController motor = new TalonFXWrapper(flywheelMotorLeft, DCMotor.getKrakenX60Foc(2), motorConfig);

	private final FlyWheelConfig flywheelConfig = new FlyWheelConfig(motor)
			.withDiameter(ShooterConstants.FLYWHEEL_DIAMETER)
			.withTelemetry("FlywheelMech", SmartMotorControllerConfig.TelemetryVerbosity.LOW)
			.withSoftLimit(ShooterConstants.FLYWHEEL_MAX_VELOCITY.times(-1), ShooterConstants.FLYWHEEL_MAX_VELOCITY)
			.withSpeedometerSimulation(ShooterConstants.FLYWHEEL_MAX_VELOCITY)
			.withMOI(ShooterConstants.FLYWHEEL_MOI);
	

	private final FlyWheel flywheel = new FlyWheel(flywheelConfig);

	public Command stop() {
		flywheel.setSpeed(RPM.of(0));
        return this.runOnce(() -> {
            motor.setVoltage(Volts.of(0));
        });
    }

	public FlywheelSubsystem()
	{
	}

	public AngularVelocity getAngularVelocity()
	{
		return flywheel.getSpeed();
	}

	public Command setAngularVelocity(AngularVelocity speed)
	{
		return flywheel.setSpeed(speed);
	}

	public Command setDutyCycle(double dutyCycle)
	{
		return flywheel.set(dutyCycle);
	}

	public Command setAngularVelocity(Supplier<AngularVelocity> speed)
	{
		return flywheel.setSpeed(speed);
	}

	public Command setDutyCycle(Supplier<Double> dutyCycle)
	{
		return flywheel.set(dutyCycle);
	}

	public Command sysId()
	{
		return flywheel.sysId(Volts.of(10), Volts.of(1).per(Second), Seconds.of(20))
		.beforeStarting(() -> SignalLogger.start()).finallyDo(() -> SignalLogger.stop());
	}

	@Override
	public void periodic()
	{
		flywheel.updateTelemetry();
		SmartDashboard.putNumber("Flywheel velocity", flywheel.getSpeed().in(RPM));
		SmartDashboard.putNumber("Flywheel velocity setpoint", motor.getMechanismSetpointVelocity().map(
			setpoint -> setpoint.in(RPM)).orElse(Double.NaN)
			);
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
