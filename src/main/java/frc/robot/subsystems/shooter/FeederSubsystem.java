package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.ShooterConstants;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;

import yams.motorcontrollers.remote.TalonFXWrapper;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class FeederSubsystem extends SubsystemBase {
	private static final TalonFX feederMotor = new TalonFX(ShooterConstants.FEEDER_ID);
	private static final Timer timer = new Timer();
	private static boolean timerEnded = true;
	private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
			.withClosedLoopController(ShooterConstants.FEEDER_KP, ShooterConstants.FEEDER_KI, ShooterConstants.FEEDER_KD, ShooterConstants.FEEDER_MAX_VELOCITY, ShooterConstants.FEEDER_MAX_ACCELERATION)
			.withGearing(ShooterConstants.FEEDER_GEARING)
			.withIdleMode(ShooterConstants.FEEDER_IDLE)
			.withMotorInverted(false)
			.withTelemetry("FeederMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
			.withStatorCurrentLimit(ShooterConstants.FEEDER_STATOR)
			.withClosedLoopRampRate(ShooterConstants.FEEDER_CLOSED_RATE)
			.withOpenLoopRampRate(ShooterConstants.FEEDER_OPEN_RATE)
			.withFeedforward(new SimpleMotorFeedforward(ShooterConstants.FEEDER_KS, ShooterConstants.FEEDER_KV, ShooterConstants.FEEDER_KA))
			.withSimFeedforward(new SimpleMotorFeedforward(ShooterConstants.FEEDER_KS, ShooterConstants.FEEDER_KV, ShooterConstants.FEEDER_KA))
			.withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP);

	private final SmartMotorController motor = new TalonFXWrapper(feederMotor, DCMotor.getKrakenX60(1), motorConfig);

	private final PivotConfig  pivotConfig = new PivotConfig(motor)
			.withSimColor(new Color8Bit(Color.kOrange))
			.withTelemetry("Feeder", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
			.withMOI(ShooterConstants.FEEDER_LENGTH, ShooterConstants.FEEDER_MASS)
			.withHardLimit(ShooterConstants.FEEDER_HARD_LIMIT_LOW, ShooterConstants.FEEDER_HARD_LIMIT_HIGH)
			.withStartingPosition(ShooterConstants.FEEDER_START_POSITION);

	private final Pivot feeder = new Pivot(pivotConfig);

	private static final DigitalInput BeamBreakLeftFeeder = new DigitalInput(ShooterConstants.BEAM_BREAK_LEFT_ID);
	private static final DigitalInput BeamBreakRightFeeder = new DigitalInput(ShooterConstants.BEAM_BREAK_RIGHT_ID);
	private static final DigitalInput BeamBreakTop = new DigitalInput(ShooterConstants.BEAM_BREAK_TOP_ID);

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
		return new ConditionalCommand(
				new SequentialCommandGroup(
						stop(),
						reset()
				),
				new SequentialCommandGroup(
						setAngle(defaultAngle),
						reset()
				),
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
		SmartDashboard.putBoolean("Feeder/Beam Break right: ", getBeamBreakRightFeeder());
		SmartDashboard.putBoolean("Feeder/Beam Break top: ", getBeamBreakTop());
		SmartDashboard.putNumber("Feeder/Feeder Angle", getAngle().in(Units.Degrees));
		feeder.updateTelemetry();
		if(BeamBreakLeftFeeder.get() || BeamBreakRightFeeder.get()){
			timer.reset();
		}
		if(timer.hasElapsed(ShooterConstants.FEEDER_TIME_PERIOD)){
			ShooterSubsystem.switchContinuous();
			timerEnded = true;
			timer.stop();
		}
	}
	public static void StartTimer(){
		ShooterSubsystem.switchContinuous();
		timerEnded = false;
		timer.stop();
		timer.reset();
	}
	public static boolean isTimerEnded(){
		return timerEnded;
	}

	@Override
	public void simulationPeriodic() {
		feeder.simIterate();
	}
}
