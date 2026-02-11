package frc.robot;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.commands.Intake;
import frc.commands.Outtake;
import frc.commands.Unstuck;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer {
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();

  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {

    // Intake bindings (POV up or A)
    m_operatorController.povUp().whileTrue(new Intake(m_intakeSubsystem));
    m_operatorController.a().whileTrue(new Intake(m_intakeSubsystem));
    
    m_operatorController.povDown().whileTrue(new Outtake(m_intakeSubsystem));

    // DEPLOY: press Right Bumper once -> deploy until deep beam break is hit -> stop automatically
    m_operatorController.rightBumper().onTrue(deployUntilDeepBeamBreak());

        // Unstuck bindings (Left bumper or X)
    m_operatorController.leftBumper().whileTrue(new Unstuck(m_intakeSubsystem));
    m_operatorController.x().whileTrue(new Unstuck(m_intakeSubsystem));
  }

  private edu.wpi.first.wpilibj2.command.Command deployUntilDeepBeamBreak() {
    return Commands.startEnd(
            () -> m_intakeSubsystem.setLinearMotorSpeed(IntakeConstants.LINEAR_INTAKE_DEPLOY_SPEED),
            m_intakeSubsystem::stopLinearMotor,
            m_intakeSubsystem)
        .until(m_intakeSubsystem::isFullyExtendedLimitSwitchTriggered)
        .withName("DeployUntilDeepBeamBreak");
  }

  public edu.wpi.first.wpilibj2.command.Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}