// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  private final static ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private final static TongueSubsystem m_tongue = new TongueSubsystem();
  public ClimberSubsystem() {
    
  }
  public ElevatorSubsystem getElevatorSubsystem(){
    return m_elevator;
  }
  public TongueSubsystem getTongueSubsystem(){
    return m_tongue;
  }
  Command l1Command = new SequentialCommandGroup(
   m_elevator.setHeight(Meters.of(ClimbConstants.TONGUE_DISTANCE)),
   m_tongue.setLength(Meters.of(0))
  );

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
