package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.ShooterConstants;

import static edu.wpi.first.hal.simulation.AnalogGyroDataJNI.getAngle;
import static edu.wpi.first.units.Units.*;

public class FeederSubsystem extends SubsystemBase {
	private static final TalonFX feederMotor = new TalonFX(ShooterConstants.FEEDER_ID);
	private final TalonFXSimState feederSim = feederMotor.getSimState();
	private final DCMotorSim feederSimModel = new DCMotorSim(
			LinearSystemId.createDCMotorSystem(
					DCMotor.getKrakenX60Foc(1),ShooterConstants.FEEDER_KA, ShooterConstants.FEEDER_GEAR_RATIO
			),
			DCMotor.getKrakenX60Foc(1)
	);
	private static final Timer timer = new Timer();
	private static boolean timerEnded = true;
	private final PositionVoltage positionRequest;
	private final VelocityVoltage velocityRequest;

	private static final DigitalInput BeamBreakLeftFeeder = new DigitalInput(ShooterConstants.BEAM_BREAK_LEFT_ID);
	private static final DigitalInput BeamBreakRightFeeder = new DigitalInput(ShooterConstants.BEAM_BREAK_RIGHT_ID);
	private static final DigitalInput BeamBreakTop = new DigitalInput(ShooterConstants.BEAM_BREAK_TOP_ID);

	public FeederSubsystem() {
		TalonFXConfiguration config = new TalonFXConfiguration();
		config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		config.CurrentLimits.StatorCurrentLimit = ShooterConstants.FEEDER_STATOR;
		config.CurrentLimits.StatorCurrentLimitEnable = true;

		config.Slot0.kP = ShooterConstants.FEEDER_KP;
		config.Slot0.kI = ShooterConstants.FEEDER_KI;
		config.Slot0.kD = ShooterConstants.FEEDER_KD;
		config.Slot0.kV = ShooterConstants.FEEDER_KV;
		config.Slot0.kA = ShooterConstants.FEEDER_KA;

		positionRequest = new PositionVoltage(0).withSlot(0);
		velocityRequest = new VelocityVoltage(1).withSlot(0);

		feederSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
	}

	public static boolean getBeamBreakLeftFeeder(){
		return BeamBreakLeftFeeder.get();
	}
	public static boolean getBeamBreakRightFeeder(){
		return BeamBreakRightFeeder.get();
	}
	public static boolean getBeamBreakTop(){
		return BeamBreakTop.get();
	}
	public void periodic() {
		SmartDashboard.putBoolean("Feeder/Beam Break left: ", getBeamBreakLeftFeeder());
		SmartDashboard.putBoolean("Feeder/Beam Break right: ", getBeamBreakLeftFeeder());
		SmartDashboard.putBoolean("Feeder/Beam Break top: ", getBeamBreakTop());
		if(BeamBreakLeftFeeder.get() || BeamBreakRightFeeder.get()){
			timer.reset();
		}
		if(timer.hasElapsed(ShooterConstants.FEEDER_TIME_PERIOD)){

			timerEnded = true;
			timer.stop();
		}
	}
	public static void StartTimer(){
		timerEnded = false;
		timer.stop();
		timer.reset();
	}
	public static boolean isHopperAlmostEmpty(){
		return timerEnded;
	}

	@Override
	public void simulationPeriodic() {
		feederSim.setSupplyVoltage(ShooterConstants.FEEDER_VOLTAGE);
		Voltage motorVoltage = feederSim.getMotorVoltageMeasure();

		feederSimModel.setInputVoltage(motorVoltage.in(Volts));
		feederSimModel.update(0.02);

		feederSim.setRawRotorPosition(feederSimModel.getAngularPosition().times(ShooterConstants.FEEDER_GEAR_RATIO));
		feederSim.setRotorVelocity(feederSimModel.getAngularVelocity().times(ShooterConstants.FEEDER_GEAR_RATIO));
	}
}
