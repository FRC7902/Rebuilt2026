package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.subsystems.intake.LinearIntakeSubsystem;

public class Autos {

	private final RobotContainer m_robotContainer;

	private final IndexerSubsystem m_indexerSubsystem;
	private final IntakeRollerSubsystem m_intakeRollerSubsystem;
	private final LinearIntakeSubsystem m_linearIntakeSubsystem;
	private final ShooterSubsystem m_shooterSubsystem;
	private final SwerveSubsystem m_swerveSubsystem;
	private final FollowPath.Builder pathBuilder;

	public Autos(RobotContainer robotContainer) {
		m_robotContainer = robotContainer;

		m_indexerSubsystem = robotContainer.m_indexerSubsystem;
		m_intakeRollerSubsystem = robotContainer.m_intakeRollerSubsystem;
		m_linearIntakeSubsystem = robotContainer.m_linearIntakeSubsystem;
		m_shooterSubsystem = robotContainer.m_shooterSubsystem;
		m_swerveSubsystem = robotContainer.m_swerveSubsystem;

		pathBuilder = new FollowPath.Builder(
				m_swerveSubsystem,
				m_swerveSubsystem::getPose,
				m_swerveSubsystem::getRobotVelocity,
				m_swerveSubsystem::drive,
				AutoConstants.TRANSLATION_PID,
				AutoConstants.ROTATION_PID,
				AutoConstants.CROSSTRACK_PID)
				.withDefaultShouldFlip()
				.withPoseReset(m_swerveSubsystem::resetOdometry);
	}

	public Command getTestPath() {
		Path myPath = new Path("back_up_right_blue");
		return pathBuilder.build(myPath);
	}
	public Command getDepotTest() {
		Path myPath = new Path("Blue_Left_Depot");
		return pathBuilder.build(myPath);
	}
	public Command getRightAutoSweepOnly(){
		Path myPath = new Path("RightAuto_Sweep");
		return pathBuilder.build(myPath);
	}
	public Command rightNeutralAutoFirstSweep() {
		return Commands.sequence(
			new InstantCommand(() -> pathBuilder.withPoseReset(m_swerveSubsystem::resetOdometry)),
			Commands.sequence(
				pathBuilder.build(new Path("RightAuto_Sweep")),
				pathBuilder.build(new Path("RightAuto_AfterSweep")) //TODO: is this necessary? Can we just have one path that goes through all the way to the end?
			).deadlineFor(
				m_intakeRollerSubsystem.intake(),
				m_indexerSubsystem.run()),
			m_swerveSubsystem.stop() // TODO: is this necessary? Does the path follower stop itself at the end of the path?
		);
	}
	public Command rightNeutralAutoSweepTwice() {
		return Commands.sequence(
			rightNeutralAutoFirstSweep()
		);
	}

}
