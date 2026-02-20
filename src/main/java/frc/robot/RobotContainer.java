// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class RobotContainer {
  
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  IntakeSubsystem m_intake = new IntakeSubsystem();
  
  public RobotContainer() {
    configureBindings();
    // Set the default command to force the elevator to go to 0.
    // m_climber.getElevatorSubsystem().setDefaultCommand(m_climber.getElevatorSubsystem().setHeight(Meters.of(0)));
    // m_climber.getTongueSubsystem().setDefaultCommand(m_climber.getTongueSubsystem().setLength(Meters.of(0)));
  }

  private void configureBindings() {
    // Schedule `setHeight` when the button 1 or 2 is pressed,
    // cancelling on release.
    // sim stuff
    m_driverController.button(1).onTrue(m_intake.setElevatorHeight(IntakeConstants.EXTEND_SETPOINT));
    m_driverController.button(2).onTrue(m_intake.setElevatorHeight(IntakeConstants.RETRACT_SETPOINT));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
