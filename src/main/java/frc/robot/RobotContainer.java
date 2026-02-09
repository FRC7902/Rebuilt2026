// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {
  private final static SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem(
    new File(Filesystem.getDeployDirectory(), "swerve"));

  private final static CommandXboxController m_driverController = new CommandXboxController(
            OperatorConstants.DRIVER_CONTROLLER_PORT);
  private final static CommandXboxController m_operatorController = new CommandXboxController(
          OperatorConstants.OPERATOR_CONTROLLER_PORT);
  
  private final AutoFactory autoFactory;
  private final AutoChooser autoChooser;

  // private static boolean isHopperEmpty = false;

  public RobotContainer() {
    configureBindings();

    autoFactory = new AutoFactory(
      m_swerveSubsystem::getPose,
      m_swerveSubsystem::resetOdometry,
      m_swerveSubsystem::followTrajectory,
      true,
      m_swerveSubsystem
    );

    // setupAuton();

    autoChooser = new AutoChooser();

    autoChooser.addRoutine("rightCycle", this::rightCycle);
    autoChooser.addRoutine("leftDepotCycle", this::leftDepotCycle);

    SmartDashboard.putData("autoChooser", autoChooser);

    RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
  }
  
  public static SwerveInputStream driveAngularVelocity = SwerveInputStream
            .of(m_swerveSubsystem.getSwerveDrive(), () -> m_driverController.getLeftY() * -1,
                    () -> m_driverController.getLeftX() * -1)
            .withControllerRotationAxis(() -> m_driverController.getRightX() * -1)
            .deadband(OperatorConstants.DEADBAND).scaleTranslation(0.8)
            .allianceRelativeControl(true);

  Command driveFieldOrientedAngularVelocity = m_swerveSubsystem.driveFieldOriented(driveAngularVelocity);

  // TODO: Finish Trajs
  private AutoRoutine rightCycle() {
    AutoRoutine routine = autoFactory.newRoutine("rightCycle");

    AutoTrajectory[] trajs = {
      routine.trajectory("Right_Neutral_Zone1"),
      routine.trajectory("Right_Shoot_Pos1"),
      routine.trajectory("Right_Neutral_Zone2"),
      routine.trajectory("Right_Shoot_Pos2"),
      routine.trajectory("Climb_Right")
    };

    routine.active().onTrue(
      Commands.sequence(
        trajs[0].resetOdometry(),
        trajs[0].cmd()
      )
    );
    trajs[0].done().onTrue(
      trajs[1].cmd()
    );
    trajs[1].done().onTrue(
      trajs[2].cmd()
    );
    trajs[2].done().onTrue(
      trajs[3].cmd()
    );
    trajs[3].done().onTrue(
      trajs[4].cmd()
    );

    return routine;
  }
  private AutoRoutine leftDepotCycle() {
    AutoRoutine routine = autoFactory.newRoutine("neutralZoneCycle");

    AutoTrajectory[] trajs = {
      routine.trajectory("Preload_Left"),
      routine.trajectory("Depot"),
      routine.trajectory("Left_Neutral_Zone_Bump"),
      routine.trajectory("Left_Shoot_Pos"),
      routine.trajectory("Climb_Left")
    };

    routine.active().onTrue(
      Commands.sequence(
        trajs[0].resetOdometry(),
        trajs[0].cmd()
      )
    );
    trajs[0].done().onTrue(
      trajs[1].cmd()
    );
    trajs[1].done().onTrue(
      trajs[2].cmd()
    );
    trajs[2].done().onTrue(
      trajs[3].cmd()
    );
    trajs[3].done().onTrue(
      trajs[4].cmd()
    );

    return routine;
  }

  private void configureBindings() {
    m_swerveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocity);
  }

  /*
  private void setupAuton() {
    autoFactory
      .bind("Shoot", new InstantCommand())  // Consider hood adjustment later
      .bind("Intake", new InstantCommand())
      .bind("Climb", new InstantCommand())
  }
  */

  /*
  @Override
  private void periodic() {
    //isHopperEmpty = shooterSubsystem.getIsHopperEmpty();
    skibidi brainrot I aaron am not skibidi
  }
  */

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
