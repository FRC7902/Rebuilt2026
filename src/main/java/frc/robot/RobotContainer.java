// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.IndexerConstants;

public class RobotContainer {
  CommandXboxController m_driverController = new CommandXboxController(0);
  Indexer m_indexer = new Indexer();
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_driverController.a().whileTrue(m_indexer.suckBalls(IndexerConstants.AGGRESIVE_MOTOR_SPEED));
    m_driverController.b().whileTrue(m_indexer.spitBalls(IndexerConstants.AGGRESIVE_MOTOR_SPEED));
    (m_driverController.b().or(m_driverController.a()))
      .whileFalse(m_indexer.stopBalls());
  }
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
