// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {
    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
    }

<<<<<<< HEAD
    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
=======
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // Intake while holding POV Up
    m_operatorController.povUp()
        .whileTrue(
            Commands.run(
                () -> m_funnelIndexerSubsystem.Intake_speed(IntakeConstants.FULL_SPEED),
                m_funnelIndexerSubsystem))
        .onFalse(
            Commands.runOnce(
                () -> m_funnelIndexerSubsystem.Intake_speed(IntakeConstants.STOP_SPEED),
                m_funnelIndexerSubsystem));

    // Outtake while holding POV Down
    m_operatorController.povDown()
        .whileTrue(
            Commands.run(
                () -> m_funnelIndexerSubsystem.Intake_speed(-IntakeConstants.OUTTAKE_SPEED),
                m_funnelIndexerSubsystem))
        .onFalse(
            Commands.runOnce(
                () -> m_funnelIndexerSubsystem.Intake_speed(IntakeConstants.STOP_SPEED),
                m_funnelIndexerSubsystem));

    // DEPLOY: press Right Bumper once -> deploy until deep beam break is hit -> stop automatically
    m_operatorController.rightBumper()
        .onTrue(deployUntilDeepBeamBreak());
  }

  private Command deployUntilDeepBeamBreak() {
    return Commands.runEnd(
            // start / execute (runs every cycle)
            () -> m_funnelIndexerSubsystem.deploy(),
            // end (runs once when finished or interrupted)
            () -> m_funnelIndexerSubsystem.stopDeploy(),
            m_funnelIndexerSubsystem
        )
        .until(() -> m_funnelIndexerSubsystem.m_isfullyExtendedLimitSwitch())
        .withName("DeployUntilDeepBeamBreak");
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
>>>>>>> 9bc822c (change the name)
