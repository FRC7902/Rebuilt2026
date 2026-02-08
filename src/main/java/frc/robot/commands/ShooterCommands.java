package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.shooter.ShooterSubsystem;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

public class ShooterCommands extends Command {
	public ShooterCommands() {
		addRequirements(RobotContainer.m_shooterSubsystem);
	}

	@Override
	public void initialize(){
		RobotContainer.m_shooterSubsystem.reset();
		RobotContainer.m_shooterSubsystem.setDefaultAngle(Degrees.of(67));
		RobotContainer.m_shooterSubsystem.setVelocitySupplier(() -> RPM.of(200));
	}
	@Override
	public void execute(){
		RobotContainer.m_shooterSubsystem.runContinuous();
	}
	@Override
	public void end(boolean interrupted){}

	@Override
	public boolean isFinished(){
		return ShooterSubsystem.isTimerEnded();
	}
}
