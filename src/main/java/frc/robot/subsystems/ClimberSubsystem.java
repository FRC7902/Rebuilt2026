// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ClimbConstants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  private final static ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private final static TongueSubsystem m_tongue = new TongueSubsystem();
  public ClimberSubsystem() {
    
  }
  public Command setElevatorHeight(Distance d){
    return m_elevator.setHeight(d);
  }
  public Command setTongueLength(Distance d){
    return m_tongue.setLength(d);
  }

  public Command getL1Command(){
    return new SequentialCommandGroup(
      m_elevator.setHeight(Meters.of(ClimbConstants.MAX_HEIGHT)), 
      m_elevator.setHeight(Meters.of(ClimbConstants.SETPOINT_3))
    );
  }

  public Command climbDown(){
    return m_elevator.setHeight(Meters.of(ClimbConstants.MAX_HEIGHT));
  }
  
  public Command getL2Command(){
    return new SequentialCommandGroup(
      getL1Command(),
      m_elevator.setHeight(Meters.of(ClimbConstants.SETPOINT_1)),
      m_elevator.setHeight(Meters.of(ClimbConstants.MAX_HEIGHT)),
      m_tongue.setLength(Meters.of(ClimbConstants.TONGUE_FULL_EXTENSION)),
      m_elevator.setHeight(Meters.of(ClimbConstants.SETPOINT_2)),
      m_tongue.setLength(Meters.of(ClimbConstants.TONGUE_INITIAL))
    );
  }

  public Command getL3Command(){
    return new SequentialCommandGroup(
      getL2Command(),
      m_elevator.setHeight(Meters.of(ClimbConstants.SETPOINT_1)),
      m_elevator.setHeight(Meters.of(ClimbConstants.MAX_HEIGHT)),
      m_tongue.setLength(Meters.of(ClimbConstants.TONGUE_FULL_EXTENSION)),
      m_elevator.setHeight(Meters.of(ClimbConstants.SETPOINT_2)),
      m_tongue.setLength(Meters.of(ClimbConstants.TONGUE_INITIAL))
    );
  }


  public Command setElevator(double dutycycle) {
    return m_elevator.set(dutycycle);
  }
  public Command setTongue(double dutycycle) {
    return m_tongue.set(dutycycle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
