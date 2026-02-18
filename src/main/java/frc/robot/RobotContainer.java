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
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;
import frc.robot.subsystems.choreo.ChoreoVars;
import frc.robot.subsystems.choreo.Routines;

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
    autoChooser = new AutoChooser();

    setupAutonomous();
    
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

    // Performs distance checks to determine which path is closer to the robot's location & should be ran
    // x is a substitute keybind for the auto trench teleop
    // TODO: Get the actual keybind when confirmed what it will be
    m_driverController.x().whileTrue(
      m_swerveSubsystem.getPose().getTranslation().getDistance(ChoreoVars.Poses.teleopNeutral.getTranslation()) <
      m_swerveSubsystem.getPose().getTranslation().getDistance(ChoreoVars.Poses.teleopAlliance.getTranslation()) ?
      new ConditionalCommand(
        autoFactory.trajectoryCmd("TeleopNeutralLeft"),
        autoFactory.trajectoryCmd("TeleopNeutralRight"),
        () -> m_swerveSubsystem.getPose().getTranslation()
        .getDistance(ChoreoVars.Poses.teleopLeftNeutralEnd.getTranslation()) < 
        m_swerveSubsystem.getPose().getTranslation()
        .getDistance(ChoreoVars.Poses.teleopRightNeutralEnd.getTranslation())
      ) :
      new ConditionalCommand(
        autoFactory.trajectoryCmd("TeleopAllianceLeft"),
        autoFactory.trajectoryCmd("TeleopAllianceRight"),
        () -> m_swerveSubsystem.getPose().getTranslation()
        .getDistance(ChoreoVars.Poses.teleopLeftAllianceEnd.getTranslation()) < 
        m_swerveSubsystem.getPose().getTranslation()
        .getDistance(ChoreoVars.Poses.teleopRightAllianceEnd.getTranslation())
      )
    );
  }

  private void setupAutonomous() {
    autoChooser.addRoutine("twoCycleRightClimb", Routines::twoCycleRightClimb)
    .addRoutine("twoCycleLeftClimb", Routines::twoCycleLeftClimb)
    .addRoutine("oneCycleRightClimb", Routines::oneCycleRightClimb)
    .addRoutine("twoCycleRight", Routines::twoCycleRight)
    .addRoutine("twoCycleLeft", Routines::twoCycleLeft)
    .addRoutine("oneCycleLeftClimb", Routines::oneCycleLeftClimb)
    .addRoutine("twoDepotCycleLeftClimb", Routines::twoDepotCycleLeftClimb)
    .addRoutine("twoDepotCycleRightClimb", Routines::twoDepotCycleRightClimb)
    .addRoutine("twoDepotCycle", Routines::twoDepotCycle)
    .addRoutine("twoDepotOneOutpostCycleRightClimb", Routines::twoDepotOneOutpostCycleRightClimb)
    .addRoutine("twoDepotOneOutpostCycleLeftClimb", Routines::twoDepotOneOutpostCycleLeftClimb)
    .addRoutine("twoDepotOneOutpostCycle", Routines::twoDepotOneOutpostCycle)
    .addRoutine("oneDepotOutpostCycleRightClimb", Routines::oneDepotOutpostCycleRightClimb)
    .addRoutine("oneDepotOutpostCycleRightClimb", Routines::oneDepotOutpostCycleLeftClimb)
    .addRoutine("oneDepotOutpostCycle", Routines::oneDepotOutpostCycle)
    .addRoutine("preloadOnly", Routines::preloadOnly);

    // autoFactory
    //   .bind("Shoot", m_shooterSubsystem.ShootCommand())  // Consider hood adjustment later
    //   .bind("Intake", m_intakeSubsystem.intake())  // Might need a whileTrue instead + check if hopper is full
    //   .bind("Climb", m_climber.ClimbToLevel())  // Needs level parameter (has to be L1)
  }


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
