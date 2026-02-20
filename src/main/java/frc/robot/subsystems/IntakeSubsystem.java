// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  private final static ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private final TalonFX m_rollerMotor;
  private final DigitalInput m_fullyRetractedLimitSwitch;
  private final DigitalInput m_fullyExtendedLimitSwitch;

  public IntakeSubsystem() {
    m_rollerMotor = new TalonFX(IntakeConstants.ROLLER_MOTOR_CAN_ID);
    m_fullyExtendedLimitSwitch = new DigitalInput(IntakeConstants.DEEP_BUTTON_BREAK_DIO);
    m_fullyRetractedLimitSwitch = new DigitalInput(IntakeConstants.SHALLOW_BUTTON_BREAK_DIO);
  }
  public boolean extendedLimitSwitchTouched(){
    return !m_fullyExtendedLimitSwitch.get();
  }
  public boolean retractedLimitSwitchTouched(){
    return !m_fullyRetractedLimitSwitch.get();
  }
  public Command intake() {
    return runOnce(() -> m_rollerMotor.set(IntakeConstants.INTAKE_SPEED));
  }
  public Command outtake() {
    return runOnce(() -> m_rollerMotor.set(IntakeConstants.OUTAKE_SPEED));
  }
  public Command stopRollers(){
    return runOnce(() -> m_rollerMotor.stopMotor());
  }
  public Command setElevatorHeight(Distance d) {
    return m_elevator.setHeight(d);
  }
  public Command intakeSequence() {
    return new SequentialCommandGroup(setElevatorHeight(Meters.of(2)));
  }
  public Command setElevator(double dutycycle) {
    return m_elevator.set(dutycycle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (extendedLimitSwitchTouched()){
      m_elevator.setElevatorPosition(IntakeConstants.EXTEND_SETPOINT);
    } else if (retractedLimitSwitchTouched()){
      m_elevator.setElevatorPosition(IntakeConstants.RETRACT_SETPOINT);
    }
  }
}
