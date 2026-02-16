package frc.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ClimbConstants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.SmartMechanism;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

/**
 * Climber subsystem for the 2026 FRC game.
 *
 * Controls a two-motor arm used to climb to fixed ladder heights.
 * Current focus is Level 1, with support for Levels 2 and 3.
 */
public class ClimbSubsystem extends SubsystemBase {

  // Left and right climber motors
  private final TalonFX m_leftMotor;

  private final TalonFX m_rightMotor;

  // Target climb height (meters)
  private double m_targetHeight = 0.0;

  private final DutyCycleOut m_dutyCycleControl;

  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      // Mechanism Circumference is the distance traveled by each mechanism rotation
      // converting rotations to meters.
      .withMechanismCircumference(Meters.of(Inches.of(0.25).in(Meters) * 22))
      // Feedback Constants (PID Constants)
      .withClosedLoopController(4, 0, 0, MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5))
      .withSimClosedLoopController(4, 0, 0, MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5))
      // Feedforward Constants
      .withFeedforward(new ElevatorFeedforward(0, 0, 0))
      .withSimFeedforward(new ElevatorFeedforward(0, 0, 0))
      // Telemetry name and verbosity level
      .withTelemetry("ElevatorMotor", TelemetryVerbosity.HIGH)
      // Gearing from the motor rotor to final shaft.
      // In this example GearBox.fromReductionStages(3,4) is the same as
      // GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to
      // your motor.
      // You could also use .withGearing(12) which does the same thing.
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
      // Motor properties to prevent over currenting.
      .withMotorInverted(false)
      .withIdleMode(MotorMode.BRAKE)
      .withStatorCurrentLimit(Amps.of(40))
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25));
  
  
  private TalonFX talon = new TalonFX(4);

  private TalonFXConfiguration config = new TalonFXConfiguration();

  private SmartMotorController talonSmartMotorController;

  private ElevatorConfig elevconfig = new ElevatorConfig(talonSmartMotorController)
      .withStartingHeight(Meters.of(0.5))
      .withHardLimits(Meters.of(0), Meters.of(3))
      .withTelemetry("Elevator", TelemetryVerbosity.HIGH)
      .withMass(Pounds.of(16));

  // Elevator Mechanism
  private Elevator elevator = new Elevator(elevconfig);

  /**
   * Creates the climber subsystem and configures motors.
   */
  public ClimbSubsystem() {
    m_leftMotor =
        new TalonFX(ClimbConstants.CLIMBER_LEFT_MOTOR_CAN_ID);

    m_rightMotor =
        new TalonFX(ClimbConstants.CLIMBER_RIGHT_MOTOR_CAN_ID);

    m_dutyCycleControl = new DutyCycleOut(0);

    configureMotor(
        m_leftMotor,
        ClimbConstants.LEFT_MOTOR_STATOR_CURRENT_LIMIT,
        ClimbConstants.LEFT_MOTOR_SUPPLY_CURRENT_LIMIT);

    configureMotor(
        m_rightMotor,
        ClimbConstants.RIGHT_MOTOR_STATOR_CURRENT_LIMIT,
        ClimbConstants.RIGHT_MOTOR_SUPPLY_CURRENT_LIMIT);

    m_targetHeight = 0.0;

    
  }

  /**
   * Applies current limits and brake mode to a motor.
   *
   * @param motor       motor to configure
   * @param statorLimit stator current limit (amps)
   * @param supplyLimit supply current limit (amps)
   */
  private void configureMotor(
      TalonFX motor,
      double statorLimit,
      double supplyLimit) {

    TalonFXConfiguration config = new TalonFXConfiguration();

    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    currentLimits.StatorCurrentLimit = statorLimit;
    currentLimits.StatorCurrentLimitEnable = true;
    currentLimits.SupplyCurrentLimit = supplyLimit;
    currentLimits.SupplyCurrentLimitEnable = true;

    config.CurrentLimits = currentLimits;

    motor.getConfigurator().apply(config);
    motor.setNeutralMode(NeutralModeValue.Brake);
    motor.setPosition(0);
  }

  /**
   * Set the height of the elevator and does not end the command when reached.
   *
   * @param angle Distance to go to.
   * @return a Command
   */
  public Command setHeight(Distance height) {
    return elevator.run(height);
  }

  /**
   * Set the height of the elevator and ends the command when reached, but not the
   * closed loop controller.
   *
   * @param angle Distance to go to.
   * @return A Command
   */
  public Command setHeightAndStop(Distance height) {
    return elevator.runTo(height, height);
  }

  /**
   * Set the elevators closed loop controller setpoint.
   *
   * @param angle Distance to go to.
   */
  public void setHeightSetpoint(Distance height) {
    elevator.setMeasurementPositionSetpoint(height);
  }

  /**
   * Move the elevator up and down.
   *
   * @param dutycycle [-1, 1] speed to set the elevator too.
   */
  public Command set(double dutycycle) {
    return elevator.set(dutycycle);
  }

  /**
   * Run sysId on the {@link Elevator}
   */
  public Command sysId() {
    return elevator.sysId(
        Volts.of(7),
        Volts.of(2).per(Second),
        Seconds.of(4));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    elevator.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    elevator.simIterate();
  }
}
