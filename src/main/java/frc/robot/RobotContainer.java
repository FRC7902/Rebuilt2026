// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;
import frc.robot.subsystems.choreo.routines;

public class RobotContainer {
  private final static SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem(
    new File(Filesystem.getDeployDirectory(), "swerve"));

  private final static CommandXboxController m_driverController = new CommandXboxController(
            OperatorConstants.DRIVER_CONTROLLER_PORT);
  private final static CommandXboxController m_operatorController = new CommandXboxController(
          OperatorConstants.OPERATOR_CONTROLLER_PORT);
  
  public static final AutoFactory autoFactory = new AutoFactory(
      m_swerveSubsystem::getPose,
      m_swerveSubsystem::resetOdometry,
      m_swerveSubsystem::followTrajectory,
      true,
      m_swerveSubsystem
    );
  private final AutoChooser autoChooser;

  // private static boolean isHopperEmpty = false;

  public RobotContainer() {
    // setupAutonomous();

    autoChooser = new AutoChooser();
    autoChooser.addRoutine("rightCycleClimb", routines::leftCycleClimb);
    autoChooser.addRoutine("leftCycleClimb", routines::rightCycleClimb);
    autoChooser.addRoutine("leftCycleDepot", routines::leftCycleDepot);
    autoChooser.addRoutine("rightCycleOutpost", routines::rightCycleOutpost);
    SmartDashboard.putData("autoChooser", autoChooser);

    RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());

    configureBindings();
  }
  
  public static SwerveInputStream driveAngularVelocity = SwerveInputStream
            .of(m_swerveSubsystem.getSwerveDrive(), () -> m_driverController.getLeftY() * -1,
                    () -> m_driverController.getLeftX() * -1)
            .withControllerRotationAxis(() -> m_driverController.getRightX() * -1)
            .deadband(OperatorConstants.DEADBAND).scaleTranslation(0.8)
            .allianceRelativeControl(true);

  Command driveFieldOrientedAngularVelocity = m_swerveSubsystem.driveFieldOriented(driveAngularVelocity);

  private void configureBindings() {
    m_swerveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocity);

    m_operatorController.start().whileTrue(m_swerveSubsystem.centerModulesCommand());

    // x is a substitute keybind for the auto trench teleop
    // TODO: Get the actual keybind when confirmed what it will be
    m_driverController.x().onTrue(
      autoFactory.trajectoryCmd("Teleop_Trench")
    );
  }

  /*
  // InstantCommands are in place for the other subsystem commands
  private void setupAutonomous() {
    autoFactory
      .bind("Shoot", new InstantCommand())  // Consider hood adjustment later
      .bind("Intake", new InstantCommand())
      .bind("ClimbL1", new InstantCommand())
      .bind("ClimbL3", new InstantCommand());
  }
  */

  /*
  @Override
  private void periodic() {
    //isHopperEmpty = shooterSubsystem.getIsHopperEmpty();
  }
  */

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
