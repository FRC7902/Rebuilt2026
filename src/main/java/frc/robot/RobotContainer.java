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
import frc.robot.subsystems.shooter.ShooterSubsystem;

import static edu.wpi.first.units.Units.Degrees;

public class RobotContainer {
	public ShooterSubsystem shooter = new ShooterSubsystem(); // holds hood, flywheel and turret
	public CommandXboxController xboxController = new CommandXboxController(0);

	public RobotContainer() {
		DriverStation.silenceJoystickConnectionWarning(true);
		configureBindings();
	}

	private void configureBindings() {
		xboxController.a().whileTrue(shooter.runShooter()).whileFalse(shooter.stopShooter());
		xboxController.b().whileTrue(shooter.runShooter(Units.DegreesPerSecond.of(720))).whileFalse(shooter.stopShooter());
		xboxController.x().whileTrue(shooter.feed(Degrees.of(90))).whileFalse(shooter.stopFeeder());
		xboxController.y().whileTrue(shooter.reset()).whileFalse(shooter.stopShooter());
	}

	public Command getAutonomousCommand() {
		return Commands.print("No autonomous command configured");
	}
}