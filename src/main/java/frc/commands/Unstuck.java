package frc.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class Unstuck extends Command {
  private final IntakeSubsystem m_intakeSubsystem;
  private boolean m_movingTowardExtended;

  /** Creates a new Unstuck command. */
  public Unstuck(IntakeSubsystem intakeSubsystem) {
    m_intakeSubsystem = intakeSubsystem;
    addRequirements(m_intakeSubsystem);
  }

  @Override
  public void initialize() {
    m_movingTowardExtended = true;
    commandLinearMotorForCurrentDirection();
  }

  @Override
  public void execute() {
    if (m_movingTowardExtended && m_intakeSubsystem.isFullyExtendedLimitSwitchTriggered()) {
      m_movingTowardExtended = false;
      commandLinearMotorForCurrentDirection();
    } else if (!m_movingTowardExtended
        && m_intakeSubsystem.isFullyRetractedLimitSwitchTriggered()) {
      m_movingTowardExtended = true;
      commandLinearMotorForCurrentDirection();
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.stopLinearMotor();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private void commandLinearMotorForCurrentDirection() {
    double speed =
        m_movingTowardExtended
            ? IntakeConstants.LINEAR_INTAKE_UNSTUCK_SPEED
            : -IntakeConstants.LINEAR_INTAKE_UNSTUCK_SPEED;
    m_intakeSubsystem.setLinearMotorSpeed(speed);
  }
}