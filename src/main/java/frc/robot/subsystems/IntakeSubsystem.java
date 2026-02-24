// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final static LinearSlide m_elevator = new LinearSlide();
  private final RollerSubsystem m_rollers = new RollerSubsystem();
  
  public IntakeSubsystem() {
    
  }
  
  public Command setElevatorHeight(Distance d) {
    return m_elevator.setHeight(d);
  }
  public Distance getElevatorHeight(){
    return m_elevator.getElevatorHeight();
  }
  public Command intakeSequence() {
    return new SequentialCommandGroup(
      setElevatorHeight(IntakeConstants.EXTEND_SETPOINT),
      m_rollers.intake());
  }
  public Command setElevator(double dutycycle) {
    return m_elevator.set(dutycycle);
  }
  public Distance getElevatorSetpoint() {
    return m_elevator.getElevatorSetpoint();
  }
  public boolean extendedLimitSwitchTouched() {
    return m_elevator.extendedLimitSwitchTouched();
  }
  public boolean getPrevTouchedELS(){
    return m_elevator.getPrevTouchedELS();
  }
  public boolean setPrevTouchedELS(boolean state){
    return m_elevator.getPrevTouchedELS();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (m_elevator.extendedLimitSwitchTouched()){
      m_elevator.setElevatorPosition(IntakeConstants.EXTEND_SETPOINT);
    } else if (m_elevator.retractedLimitSwitchTouched()){
      m_elevator.setElevatorPosition(IntakeConstants.RETRACT_SETPOINT);
    }
  }
}
