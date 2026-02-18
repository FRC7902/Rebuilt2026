// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.subsystems.ClimbSubsystem;

public class RobotContainer {

  private final CommandXboxController m_driverController = new CommandXboxController(0);
  ClimbSubsystem m_climber = new ClimbSubsystem();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {

    m_driverController.button(1).onTrue(m_climber.climbL1());
    m_driverController.button(2).onTrue(m_climber.climbL2());
    m_driverController.button(3).onTrue(m_climber.climbL3());


  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
