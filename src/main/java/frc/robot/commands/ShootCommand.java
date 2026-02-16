package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

public class ShootCommand extends Command {
	ShooterSubsystem m_shooterSubsystem;
	public ShootCommand(ShooterSubsystem shooterSubsystem) {
		m_shooterSubsystem = shooterSubsystem;
		addRequirements(shooterSubsystem);
	}

	@Override
	public void initialize(){
		m_shooterSubsystem.setDefaultAngle(Degrees.of(67));
		m_shooterSubsystem.setVelocitySupplier(() -> RPM.of(200));
	}
	@Override
	public void execute(){
		m_shooterSubsystem.runFeeder();
	}
	@Override
	public void end(boolean interrupted){}

	@Override
	public boolean isFinished(){
		return ShooterSubsystem.isHopperAlmostEmpty();
	}
}
