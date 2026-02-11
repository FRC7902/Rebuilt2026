// the WPILib BSD license file in the root directory of this project.

package frc.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class Intake extends Command {
  private final IntakeSubsystem m_intakeSubsystem;

  /** Creates a new Intake command. */
  public Intake(IntakeSubsystem intakeSubsystem) {
    m_intakeSubsystem = intakeSubsystem;
    addRequirements(m_intakeSubsystem);
  }

  @Override
  public void initialize() {
    m_intakeSubsystem.setSpeed(IntakeConstants.FULL_SPEED);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.stopIntake();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}