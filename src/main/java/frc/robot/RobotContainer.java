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
	public static HoodSubsystem hoodSubsystem = new HoodSubsystem();
	public static FeederSubsystem feederSubsystem = new FeederSubsystem();
	public CommandXboxController xboxController = new CommandXboxController(0);
	ShootCommand shooterCommand;

	public RobotContainer() {
		DriverStation.silenceJoystickConnectionWarning(true);
		configureBindings();
	}

	private void configureBindings() {
		//TODO: Fix hood, setAngle in yams library does not work currently
		// Test hood movement
		xboxController.a().whileTrue(hoodSubsystem.setAngle(Degrees.of(75))).whileFalse(hoodSubsystem.idle());
		xboxController.b().whileTrue(hoodSubsystem.setAngle(Degrees.of(15))).whileFalse(hoodSubsystem.idle());

		//test feeder
		xboxController.x().whileTrue(new InstantCommand(()-> m_shooterSubsystem.runFeeder(1000)));
		//test Flywheel
		xboxController.y().whileTrue(m_shooterSubsystem.runShooter(RotationsPerSecond.of(0.5))).whileFalse(m_shooterSubsystem.stopShooter());
	}

	public Command getAutonomousCommand() {
		return Commands.print("No autonomous command configured");
	}
}