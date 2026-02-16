// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.shooter.FeederSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

import static edu.wpi.first.units.Units.*;

public class RobotContainer {
	public static ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem(); // holds hood, flywheel and turret
	public CommandXboxController xboxController = new CommandXboxController(0);

	public RobotContainer() {
		DriverStation.silenceJoystickConnectionWarning(true);
		configureBindings();
	}

	private void configureBindings() {

		//test feeder
		xboxController.x().whileTrue(Commands.run(()-> m_shooterSubsystem.runFeeder())).whileFalse(Commands.run(()->m_shooterSubsystem.stopFeeder()));
		//test Flywheel
		xboxController.y().whileTrue(m_shooterSubsystem.runShooter(RotationsPerSecond.of(0.5))).whileFalse(m_shooterSubsystem.stopShooter());
	}

	public Command getAutonomousCommand() {
		return Commands.print("No autonomous command configured");
	}
}