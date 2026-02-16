// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import javax.naming.PartialResultException;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.ClimberSubsystem;

public class RobotContainer {
  public final static ClimberSubsystem m_climber = new ClimberSubsystem();
  private final CommandXboxController m_driverController = new CommandXboxController(0);

  Command l1Command = new SequentialCommandGroup(
    m_climber.setHeight(Meters.of(ClimbConstants.TRAVEL_DISTANCE)),
    m_climber.setHeight(Meters.of(0))
  );
  
  public RobotContainer() {
    configureBindings();
    // Set the default command to force the elevator to go to 0.
    m_climber.setDefaultCommand(m_climber.setHeight(Meters.of(0)));
  }

  private void configureBindings() {
    // Schedule `setHeight` when the button 1 or 2 is pressed,
    // cancelling on release.
    // sim stuff
    m_driverController.button(1).whileTrue(m_climber.setHeight(Meters.of(0.5)));
    m_driverController.button(2).whileTrue(m_climber.setHeight(Meters.of(1)));
    // Schedule `set` when button 3 is pressed,
    // cancelling on release.
    m_driverController.button(3).whileTrue(m_climber.set(0.3));
    m_driverController.button(4).whileTrue(m_climber.set(-0.3));
    
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
