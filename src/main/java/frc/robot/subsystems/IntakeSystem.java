package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.subsystems.intake.LinearIntakeSubsystem;
import frc.robot.subsystems.intake.LinearIntakeSubsystem.LinearIntakePosition;

public class IntakeSystem extends SubsystemBase {

    private final IntakeRollerSubsystem m_intakeRollerSubsystem;
    private final LinearIntakeSubsystem m_linearIntakeSubsystem;

    public IntakeSystem() {
        m_intakeRollerSubsystem = new IntakeRollerSubsystem();
        m_linearIntakeSubsystem = new LinearIntakeSubsystem();
    }

    public Command stop() {
        return midpoint().andThen(stopRoller());
    }

    public Command stopRoller() {
        return m_intakeRollerSubsystem.stop();
    }

    public Command intake() {
        return m_intakeRollerSubsystem.intake();
    }

    public Command outtake() {
        return m_intakeRollerSubsystem.outtake();
    }

    public Command extend() {
        return m_linearIntakeSubsystem.extend();
    }

    public Command midpoint() {
        return m_linearIntakeSubsystem.midpoint();
    }

    public Command retract() {
        return m_linearIntakeSubsystem.retract();
    }

    public Command shuffle() {
        return m_linearIntakeSubsystem.shuffle();
    }

    public LinearIntakePosition getCurrentPositionEnum() {
        return m_linearIntakeSubsystem.getCurrentPositionEnum();
    }

    public void calibrateLinearIntakePosition() {
        m_linearIntakeSubsystem.calibrateLinearIntakePosition();
    }
}