// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.FlyWheelConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LinearIntakeSubsystem;
import frc.robot.subsystems.Rollers;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.shooter.FeederSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {
  public final static SwerveSubsystem drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve"));
  CommandXboxController m_driverController = new CommandXboxController(0);
  LinearIntakeSubsystem m_linearSlide = new LinearIntakeSubsystem();
  Rollers m_rollers = new Rollers();
  public static IndexerSubsystem m_indexer = new IndexerSubsystem();
  ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  FeederSubsystem feederSubsystem = new FeederSubsystem();

  // Establish a Sendable Chooser that will be able to be sent to the SmartDashboard, allowing selection of desired auto
  private final SendableChooser<Command> autoChooser;

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> m_driverController.getLeftY() * 1,
                                                                () -> m_driverController.getLeftX() * 1)
                                                            .withControllerRotationAxis(() -> m_driverController.getRightX() * -1)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);
  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(m_driverController::getRightX,
                                                                                             m_driverController::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -m_driverController.getLeftY(),
                                                                        () -> -m_driverController.getLeftX())
                                                                    .withControllerRotationAxis(() -> m_driverController.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  m_driverController.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  m_driverController.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true)
                                                                               .translationHeadingOffset(true)
                                                                               .translationHeadingOffset(Rotation2d.fromDegrees(
                                                                                   0));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    if (Robot.isSimulation()){
      DriverStation.silenceJoystickConnectionWarning(true);
    }
    //Have the autoChooser pull in all PathPlanner autos as options
    autoChooser = AutoBuilder.buildAutoChooser();

    //Set the default auto (do nothing) 
    autoChooser.setDefaultOption("Do Nothing", Commands.none());

    //Add a simple auto option to have the robot drive forward for 1 second then stop
    autoChooser.addOption("Drive Forward", drivebase.driveForward().withTimeout(1));
    
    //Put the autoChooser on the SmartDashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }
  
  private void configureBindings()
  {
    Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    
    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);

    m_driverController.povUp().whileTrue(drivebase.driveForward());
    m_driverController.povDown().whileTrue(drivebase.driveBackward());
    m_driverController.povLeft().whileTrue(drivebase.driveLeft());
    m_driverController.povRight().whileTrue(drivebase.driveRight());  
    m_driverController.start().onTrue(new InstantCommand(() -> drivebase.zeroGyro()));

    m_driverController.leftBumper().whileTrue(m_rollers.intake()).onFalse(m_rollers.stopRollers());
    m_driverController.a().onTrue(m_shooterSubsystem.aimAt(Degrees.of(30)));
		m_driverController.b().onTrue(m_shooterSubsystem.aimAt(Degrees.of(9)));

    //Flywheel binding
		m_driverController.x().onTrue(m_shooterSubsystem.runShooter(FlyWheelConstants.FLYWHEEL_TARGET_ANGULAR_VELOCITY));
    m_driverController.y().onTrue(m_shooterSubsystem.stopFlyWheel());
		// // Feeder binding
		m_driverController.rightTrigger().onTrue(new InstantCommand(() -> m_shooterSubsystem.runFeeder(FeederConstants.FEEDER_TARGET_SPEED)));
    m_driverController.leftTrigger().onTrue(feederSubsystem.stop());
  }
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
