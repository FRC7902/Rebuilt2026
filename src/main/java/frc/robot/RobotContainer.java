package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
<<<<<<< HEAD

public class RobotContainer {
    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
    }
=======
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
>>>>>>> d54fe03 (added the intake, outtake and unstuck command. and fixes all of the bugs)

<<<<<<< HEAD
    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
=======
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {

    // Intake bindings (POV up or A)
    m_operatorController.povUp().whileTrue(new Intake(m_intakeSubsystem));
    m_operatorController.a().whileTrue(new Intake(m_intakeSubsystem));
    
    m_operatorController.povDown().whileTrue(new Outtake(m_intakeSubsystem));
        // Unstuck bindings (Left bumper or X)
    m_operatorController.leftBumper().whileTrue(new Unstuck(m_intakeSubsystem));
    m_operatorController.x().whileTrue(new Unstuck(m_intakeSubsystem));
  }

  public IntakeSubsystem getIntakeSubsystem() {
    return m_intakeSubsystem;
  }

  public edu.wpi.first.wpilibj2.command.Command getAutonomousCommand() {
    return null;
  }
<<<<<<< HEAD
}
>>>>>>> 9bc822c (change the name)
=======
}
>>>>>>> d54fe03 (added the intake, outtake and unstuck command. and fixes all of the bugs)
