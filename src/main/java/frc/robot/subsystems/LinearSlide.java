// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.Optional;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class LinearSlide extends SubsystemBase {
  // Vendor motor controller object
  private final TalonFX m_linearMotor = new TalonFX(IntakeConstants.LINEAR_MOTOR_CAN_ID);
  private final DigitalInput m_fullyRetractedLimitSwitch;
  private final DigitalInput m_fullyExtendedLimitSwitch;

  // Motor configs for elevator motor
  private final SmartMotorControllerConfig linearConfig = new SmartMotorControllerConfig(this)
  .withControlMode(ControlMode.CLOSED_LOOP)
  // Mechanism Circumference is the distance traveled by each mechanism rotation converting rotations to meters.
  .withMechanismCircumference(IntakeConstants.MECH_CIRCUMFERENCE)
  // Feedback Constants (PID Constants)
  .withClosedLoopController(IntakeConstants.Linear_kP, IntakeConstants.Linear_kI, IntakeConstants.Linear_kD, IntakeConstants.MAX_VELOCITY,IntakeConstants.MAX_ACCELERATION)
  .withSimClosedLoopController(IntakeConstants.Sim_kP, IntakeConstants.Sim_kI, IntakeConstants.Sim_kP, IntakeConstants.MAX_VELOCITY, IntakeConstants.MAX_ACCELERATION)
  // Feedforward Constants
  .withFeedforward(IntakeConstants.FEED_FORWARD)
  .withSimFeedforward(new ElevatorFeedforward(IntakeConstants.Linear_kS, IntakeConstants.Linear_kG, IntakeConstants.Linear_kV))
  // Telemetry name and verbosity level
  .withTelemetry("ElevatorMotor", TelemetryVerbosity.HIGH)
  // Gearing from the motor rotor to final shaft.
  // In this example GearBox.fromReductionStages(3,4) is the same as GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your motor.
  // You could also use .withGearing(12) which does the same thing.
  .withGearing(new MechanismGearing(IntakeConstants.GEARBOX))
  // Motor properties to prevent over currenting.
  .withMotorInverted(IntakeConstants.MOTOR_INVERTED)
  .withIdleMode(MotorMode.BRAKE)
  .withStatorCurrentLimit(IntakeConstants.STATOR_CURRENT_LIMIT)
  .withClosedLoopRampRate(IntakeConstants.CLOSED_LOOP_RAMP_RATE)
  .withOpenLoopRampRate(IntakeConstants.OPEN_LOOP_RAMP_RATE);


  // SmartMotorController using TalonFX Wrapper
  private SmartMotorController linearMotorController = new TalonFXWrapper(m_linearMotor, DCMotor.getFalcon500Foc(1), linearConfig);


  private ElevatorConfig linearMotorconfig = new ElevatorConfig(linearMotorController)
      .withStartingHeight(IntakeConstants.STARTING_HEIGHT)
      .withHardLimits(IntakeConstants.MIN_HARD_LIMIT, IntakeConstants.MAX_HARD_LIMIT)
      .withTelemetry("Elevator", TelemetryVerbosity.HIGH)
      .withMass(IntakeConstants.DLI_MASS)
      .withAngle(IntakeConstants.DLI_ANGLE);
  // Elevator Mechanism
  private Elevator elevator = new Elevator(linearMotorconfig);
  /**
   * Set the height of the elevator and does not end the command when reached.
   * @param angle Distance to go to.
   * @return a Command
   */
  public Command setHeight(Distance height) { 
    // default is 0 and it keeps resetting
    return elevator.runTo(height,Meters.of(IntakeConstants.LINEAR_TOLERANCE));
  }

  /**
   * Set the elevators closed loop controller setpoint.
   * @param angle Distance to go to.
   */
  public void setHeightSetpoint(Distance height) { 
    elevator.setMeasurementPositionSetpoint(height);
  }

  /**
   * Move the elevator up and down.
   * @param dutycycle [-1, 1] speed to set the elevator too.
   */
  public Command set(double dutycycle) { return elevator.set(dutycycle);}

  /**
   * Run sysId on the {@link Elevator}
   */
  public Command sysId() { return 
    elevator.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));
  }

  public Command protectIntake(){
    return new ConditionalCommand(
      new InstantCommand(() -> {
        setHeight(IntakeConstants.RETRACT_SETPOINT);
      }),
      Commands.none(), 
      () -> !extendedLimitSwitchTouched() && getElevatorSetpoint().equals(IntakeConstants.EXTEND_SETPOINT)
    ).repeatedly();
  }

  //Gets the elevator's setpoint as a distance
  public Distance getElevatorSetpoint(){
    Optional<Angle> angle_position = elevator.getMechanismSetpoint();
    if (!angle_position.isPresent()){
      return null;
    }
    return linearConfig.convertFromMechanism(angle_position.get());
  }
  // Corrects the elevator position
  public void setElevatorPosition(Distance actualPosition){
    Angle encoderPosition = linearConfig.convertToMechanism(actualPosition);
    linearMotorController.setEncoderPosition(encoderPosition);
  }

  public Distance getElevatorHeight(){
    return elevator.getHeight();
  }

  /** Creates a new LinearSlide. */
  public LinearSlide() {
    m_fullyExtendedLimitSwitch = new DigitalInput(IntakeConstants.EXTENDED_LIMIT_SWITCH_DIO);
    m_fullyRetractedLimitSwitch = new DigitalInput(IntakeConstants.RETRACTED_LIMIT_SWITCH_DIO);
  }
  public boolean extendedLimitSwitchTouched(){
    return m_fullyExtendedLimitSwitch.get(); //TODO: Need to check behaviour
  }
  public boolean retractedLimitSwitchTouched(){
    return m_fullyRetractedLimitSwitch.get(); //TODO: Need to check behaviour
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    elevator.updateTelemetry();
  }
  @Override
  public void simulationPeriodic() {
    elevator.simIterate();
  }
}