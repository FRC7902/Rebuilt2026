// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.LEDSubsystem;

public class RobotContainer {

  CommandXboxController m_driverController = new CommandXboxController(0);
  public RobotContainer() {
    configureBindings();
    LEDSubsystem.getInstance().setDefaultCommand(LEDSubsystem.getInstance().setLedPattern(LEDPattern.kOff));
  }

  private void configureBindings() {
    m_driverController.a().whileTrue(LEDSubsystem.getInstance().setLedPattern(LEDPattern.solid(Color.kLightCyan)));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
