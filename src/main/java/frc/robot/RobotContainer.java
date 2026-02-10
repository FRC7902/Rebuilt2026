// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

public class RobotContainer {
	public static ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem(); // holds hood, flywheel and turret
	public static FlywheelSubsystem flywheelSubsystem = new FlywheelSubsystem();
	public CommandXboxController xboxController = new CommandXboxController(0);

	public RobotContainer() {
		DriverStation.silenceJoystickConnectionWarning(true);
		configureBindings();
	}

	private void configureBindings() {
		// Test hood movement
		xboxController.a().whileTrue(m_shooterSubsystem.aimAt(Degrees.of(45)));
		xboxController.b().whileTrue(m_shooterSubsystem.aimAt(Degrees.of(90)));

		// Test flywheel movement
//		xboxController.x().whileTrue(flywheelSubsystem.setDutyCycle(0.5)); // 50% power
		xboxController.y().whileTrue(flywheelSubsystem.setVelocity(RPM.of(1000))).whileFalse(flywheelSubsystem.setVelocity(RPM.of(0)));
	}

	public Command getAutonomousCommand() {
		return Commands.print("No autonomous command configured");
	}
}