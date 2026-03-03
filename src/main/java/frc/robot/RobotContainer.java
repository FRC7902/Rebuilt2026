// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

import edu.wpi.first.units.measure.Angle;
import static edu.wpi.first.units.Units.Degrees;

import java.io.File;
import swervelib.SwerveInputStream;

import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.LinearSlide;
import frc.robot.subsystems.Rollers;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;

public class RobotContainer {
  public final static SwerveSubsystem drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve"));
  CommandXboxController m_driverController = new CommandXboxController(0);
  LinearSlide m_linearSlide = new LinearSlide();
  Rollers m_rollers = new Rollers();
  Indexer m_indexer = new Indexer();
  ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();

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
    DriverStation.silenceJoystickConnectionWarning(true);
    
    //Create the NamedCommands that will be used in PathPlanner
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

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
    
    m_indexer.setDefaultCommand(m_indexer.suckBalls(IndexerConstants.AGGRESIVE_MOTOR_SPEED));
    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);

    // test bindings, more to be added later
    // m_driverController.circle().onTrue(DLI.setHeight(Meters.of(Units.inchesToMeters(13))));
    // m_driverController.cross().onTrue(DLI.setHeight(Meters.of(0)));
    // intaking sequence
    // m_driverController.triangle().onTrue(new SequentialCommandGroup(
    //   DLI.setHeight(IntakeConstants.EXTEND_SETPOINT),
    //   rollers.intake()
    // ));
    // // retracting sequence
    // m_driverController.cross().onTrue(new SequentialCommandGroup(
    //   rollers.stopRollers(),
    //   DLI.setHeight(IntakeConstants.RETRACT_SETPOINT)
    // ));
    // DLI.protectIntake();
    m_driverController.povUp().whileTrue(drivebase.driveForward());
    m_driverController.povDown().whileTrue(drivebase.driveBackward());
    m_driverController.povLeft().whileTrue(drivebase.driveLeft());
    m_driverController.povRight().whileTrue(drivebase.driveRight());  
    m_driverController.start().onTrue(new InstantCommand(() -> drivebase.zeroGyro()));

  //   m_driverController.leftBumper().whileTrue(
  //     new ParallelCommandGroup(
  //       DLI.setHeight(IntakeConstants.EXTEND_SETPOINT),
  //       rollers.intake()))
  //       .onFalse(new SequentialCommandGroup(
  //       rollers.stopRollers()));
  //   m_driverController.rightBumper().onTrue(DLI.setHeight(IntakeConstants.RETRACT_SETPOINT));
    m_driverController.leftBumper().whileTrue(m_rollers.intake()).onFalse(m_rollers.stopRollers());
    m_driverController.a().onTrue(m_shooterSubsystem.aimAt(Degrees.of(30)));
		m_driverController.b().onTrue(m_shooterSubsystem.aimAt(Degrees.of(9)));

    //Flywheel binding
		m_driverController.x().onTrue(m_shooterSubsystem.runShooter(ShooterConstants.FLYWHEEL_TARGET_ANGULAR_VELOCITY));
    m_driverController.y().onTrue(m_shooterSubsystem.stopFlyWheel());
		// // Feeder binding
		m_driverController.leftBumper().onTrue(new InstantCommand(() -> m_shooterSubsystem.runFeeder(ShooterConstants.FEEDER_TARGET_SPEED)));
    m_driverController.rightBumper().onTrue(new InstantCommand(() -> m_shooterSubsystem.stopFeeder()) );
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
