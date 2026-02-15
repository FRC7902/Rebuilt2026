package frc.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public final class Unstuck {
  private Unstuck() {}

  /**
   * Moves half-out then half-in using closed-loop position control.
   *
   * <p>PID output is naturally larger when far from target and gets smaller as it approaches the
   * target, so this starts faster and slows near the stop point.
   */
  public static Command halfOutHalfIn(IntakeSubsystem intakeSubsystem) {
    return Commands.sequence(
            Commands.runOnce(() -> intakeSubsystem.setLinearIntakeTargetPosition(1.0), intakeSubsystem),
            Commands.waitUntil(intakeSubsystem::isLinearIntakeAtTargetPosition)
                .withTimeout(IntakeConstants.UNSTUCK_HALF_CYCLE_TIMEOUT_SECONDS),
            Commands.waitSeconds(IntakeConstants.UNSTUCK_SETTLE_TIME_SECONDS),
            Commands.runOnce(() -> intakeSubsystem.setLinearIntakeTargetPosition(0.0), intakeSubsystem),
            Commands.waitUntil(intakeSubsystem::isLinearIntakeAtTargetPosition)
                .withTimeout(IntakeConstants.UNSTUCK_HALF_CYCLE_TIMEOUT_SECONDS),
            Commands.waitSeconds(IntakeConstants.UNSTUCK_SETTLE_TIME_SECONDS))
        .finallyDo((interrupted) -> intakeSubsystem.stopLinearIntake())
        .withName("UnstuckHalfOutHalfInPid");
  }
}