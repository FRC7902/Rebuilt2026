package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.subsystems.intake.LinearIntakeSubsystem;
import swervelib.SwerveInputStream;

public class Autos {

	private final RobotContainer m_robotContainer;

	private final IndexerSubsystem m_indexerSubsystem;
	private final IntakeRollerSubsystem m_intakeRollerSubsystem;
	private final LinearIntakeSubsystem m_linearIntakeSubsystem;
	private final ShooterSubsystem m_shooterSubsystem;
	private final SwerveSubsystem m_swerveSubsystem;
	private final FollowPath.Builder pathBuilder;

	public final SwerveInputStream stationaryAutoAim;
	private final DoubleSupplier m_autoAimHeadingX;
	private final DoubleSupplier m_autoAimHeadingY;

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

		m_autoAimHeadingX = robotContainer.autoAimHeadingX();
		m_autoAimHeadingY = robotContainer.autoAimHeadingY();

		stationaryAutoAim = SwerveInputStream.of(m_swerveSubsystem.getSwerveDrive(),
				() -> 0.0,
				() -> 0.0)
				.deadband(OperatorConstants.DEADBAND)
				.scaleTranslation(1.0)
				.allianceRelativeControl(true)
				.withControllerHeadingAxis(m_autoAimHeadingX, m_autoAimHeadingY)
				.headingWhile(true)
				.scaleTranslation(SwerveConstants.AUTO_AIM_SCALE_TRANSLATION);
	}

	public Command rightAutoSweepOnly() {
		Path myPath = new Path("RightAuto_Sweep");
		return pathBuilder.build(myPath);
	}

	public Command rightNeutralAutoFirstSweep() {
		return Commands.sequence(
				new InstantCommand(() -> pathBuilder.withPoseReset(m_swerveSubsystem::resetOdometry)),
				Commands.sequence(
						pathBuilder.build(new Path("RightAuto_Sweep")),
						pathBuilder.build(new Path("RightAuto_AfterSweep")) // TODO: is this necessary? Can we just have
																			// one path that goes through all the way to
																			// the end?
				).deadlineFor(
						m_intakeRollerSubsystem.intake(),
						m_indexerSubsystem.run()),
				m_swerveSubsystem.stop() // TODO: is this necessary? Does the path follower stop itself at the end of
											// the path?
		);
	}

	public Command rightNeutralAutoSweepTwice() {
		return Commands.sequence(
				rightNeutralAutoFirstSweep());
	}

	public Command shootPreloadAuto() {
		return Commands.sequence(
				new InstantCommand(() -> pathBuilder.withPoseReset(m_swerveSubsystem::resetOdometry)),
				pathBuilder.build(new Path("Backup_Preload")),
				Commands.parallel(
						m_swerveSubsystem.driveFieldOriented(stationaryAutoAim),
						m_shooterSubsystem.aimAndShootIgnoreCheck(() -> m_swerveSubsystem.getDistanceToTarget()),
						m_linearIntakeSubsystem.shuffle()));
	}

	public Command depotAuto() {
		return Commands.sequence(
				shootPreloadAuto(),
				pathBuilder.build(new Path("Preload_to_Depot")).deadlineFor(
						m_intakeRollerSubsystem.intake(),
						m_indexerSubsystem.run()),
				Commands.waitSeconds(3).deadlineFor(
						m_intakeRollerSubsystem.intake(),
						m_indexerSubsystem.run()),
				pathBuilder.build(new Path("Forward_From_Depot")),
				m_swerveSubsystem.stop(),
				Commands.parallel(
						m_swerveSubsystem.driveFieldOriented(stationaryAutoAim),
						m_shooterSubsystem.aimAndShootIgnoreCheck(() -> m_swerveSubsystem.getDistanceToTarget()),
						m_linearIntakeSubsystem.shuffle()));
	}

	public Command depotPath() {
		return Commands.sequence(
			pathBuilder.build(new Path("Backup_Preload")),
			pathBuilder.build(new Path("Preload_to_Depot")),
			pathBuilder.build(new Path("Forward_From_Depot"))
		);
	}

}
