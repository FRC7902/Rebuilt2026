package frc.robot;

import java.util.function.DoubleSupplier;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.Constants.ClimbConstants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.IndexerSystem;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.ShooterSystem;
import frc.robot.subsystems.SwerveSystem;
import frc.robot.subsystems.climb.ElevatorSubsystem;
import swervelib.SwerveInputStream;

public class Choreo {

        private final AutoFactory m_autoFactory;

        private final RobotContainer m_robotContainer;

        private final IndexerSystem m_indexerSystem;
        private final IntakeSystem m_intakeSystem;
        private final ShooterSystem m_shooterSystem;
        private final SwerveSystem m_swerveSystem;
        private final ElevatorSubsystem m_elevatorSubsystem;

        private final DoubleSupplier m_autoAimHeadingX;
        private final DoubleSupplier m_autoAimHeadingY;

        public final SwerveInputStream stationaryAutoAim;
        public final SwerveInputStream aimTowardsRed;
        public final SwerveInputStream aimTowardsBlue;

        public final SwerveInputStream backUpAndAimRed;
        public final SwerveInputStream backUpAndAimBlue;

        public Choreo(RobotContainer robotContainer) {
                m_robotContainer = robotContainer;

                m_autoFactory = robotContainer.m_autoFactory;

                m_indexerSystem = robotContainer.m_indexerSystem;
                m_intakeSystem = robotContainer.m_intakeSystem;
                m_shooterSystem = robotContainer.m_shooterSystem;
                m_swerveSystem = robotContainer.m_swerveSystem;
                m_elevatorSubsystem = robotContainer.m_elevatorSubsystem;

                m_autoAimHeadingX = robotContainer.autoAimHeadingX();
                m_autoAimHeadingY = robotContainer.autoAimHeadingY();

                stationaryAutoAim = SwerveInputStream.of(m_swerveSystem.getSwerveDrive(),
                                () -> 0.0,
                                () -> 0.0)
                                .deadband(OperatorConstants.DEADBAND)
                                .scaleTranslation(1.0)
                                .allianceRelativeControl(true)
                                .withControllerHeadingAxis(m_autoAimHeadingX, m_autoAimHeadingY)
                                .headingWhile(true)
                                .scaleTranslation(SwerveConstants.AUTO_AIM_SCALE_TRANSLATION);

                aimTowardsRed = stationaryAutoAim.copy()
                                .withControllerHeadingAxis(() -> 0.0, () -> -1.0);

                aimTowardsBlue = stationaryAutoAim.copy()
                                .withControllerHeadingAxis(() -> 0.0, () -> 1.0);

                backUpAndAimRed = SwerveInputStream.of(m_swerveSystem.getSwerveDrive(),
                                () -> -0.25,
                                () -> 0.0)
                                .deadband(OperatorConstants.DEADBAND)
                                .scaleTranslation(1.0)
                                .allianceRelativeControl(true)
                                .withControllerHeadingAxis(m_autoAimHeadingX, m_autoAimHeadingY)
                                .headingWhile(true)
                                .scaleTranslation(SwerveConstants.AUTO_AIM_SCALE_TRANSLATION);

                backUpAndAimBlue = SwerveInputStream.of(m_swerveSystem.getSwerveDrive(),
                                () -> 0.25,
                                () -> 0.0)
                                .deadband(OperatorConstants.DEADBAND)
                                .scaleTranslation(1.0)
                                .allianceRelativeControl(true)
                                .withControllerHeadingAxis(m_autoAimHeadingX, m_autoAimHeadingY)
                                .headingWhile(true)
                                .scaleTranslation(SwerveConstants.AUTO_AIM_SCALE_TRANSLATION);
        }

        public Command shootPreloadAuto() {
                return Commands.sequence(
                                // m_autoFactory.resetOdometry("ShootPreloadAuto"),
                                // m_autoFactory.trajectoryCmd("ShootPreloadAuto"),
                                // new InstantCommand(
                                // () -> m_swerveSubsystem.drive(new Translation2d(-0.25, 0.0), 0, false)),
                                Commands.waitSeconds(3).deadlineFor(new ConditionalCommand(
                                                m_swerveSystem.driveFieldOriented(backUpAndAimRed),
                                                m_swerveSystem.driveFieldOriented(backUpAndAimBlue),
                                                m_swerveSystem::isRedAlliance)),
                                m_swerveSystem.stop(),
                                Commands.parallel(
                                                m_swerveSystem.driveFieldOriented(stationaryAutoAim),
                                                m_indexerSystem.run(),
                                                m_intakeSystem.intake(),
                                                m_shooterSystem.aimAndShootIgnoreCheck(
                                                                () -> m_swerveSystem.getDistanceToTarget(true))));
        }

        private Command rightNeutralAutoFirstSweep() {
                return Commands.sequence(
                                m_autoFactory.resetOdometry("RightAuto1"),
                                m_autoFactory.trajectoryCmd("RightAuto1").deadlineFor(
                                                m_intakeSystem.intake(),
                                                m_indexerSystem.run()),
                                m_swerveSystem.stop());
        }

        private Command leftNeutralAutoFirstSweep() {
                return Commands.sequence(
                                m_autoFactory.resetOdometry("LeftAuto1"),
                                m_autoFactory.trajectoryCmd("LeftAuto1").deadlineFor(
                                                m_intakeSystem.intake(),
                                                m_indexerSystem.run()),
                                m_swerveSystem.stop());
        }

        public Command leftNeutralAutoSweepOnce() {
                return Commands.sequence(
                                leftNeutralAutoFirstSweep(),
                                m_autoFactory.trajectoryCmd("LeftAuto2a"),
                                m_swerveSystem.stop(),
                                Commands.parallel(
                                                m_swerveSystem.driveFieldOriented(stationaryAutoAim),
                                                m_shooterSystem.aimAndShootIgnoreCheck(
                                                                () -> m_swerveSystem.getDistanceToTarget(true)),
                                                m_intakeSystem.shuffle()));
        }

        public Command rightNeutralAutoSweepOnce() {
                return Commands.sequence(
                                rightNeutralAutoFirstSweep(),
                                m_autoFactory.trajectoryCmd("RightAuto2a"),
                                m_swerveSystem.stop(),
                                Commands.parallel(
                                                m_swerveSystem.driveFieldOriented(stationaryAutoAim),
                                                m_shooterSystem.aimAndShootIgnoreCheck(
                                                                () -> m_swerveSystem.getDistanceToTarget(true)),
                                                m_intakeSystem.shuffle()));
        }

        public Command leftNeutralAutoSweepTwice() {
                return Commands.sequence(
                                leftNeutralAutoFirstSweep(),
                                m_autoFactory.trajectoryCmd("LeftAuto2a"),
                                m_swerveSystem.stop(),
                                Commands.waitSeconds(8).deadlineFor(
                                                m_swerveSystem.driveFieldOriented(stationaryAutoAim),
                                                m_shooterSystem.aimAndShootIgnoreCheck(
                                                                () -> m_swerveSystem.getDistanceToTarget(true)),
                                                m_intakeSystem.shuffle()),
                                m_autoFactory.trajectoryCmd("LeftAuto3a").deadlineFor(
                                                m_shooterSystem.stopShooting(),
                                                m_intakeSystem.extend()),
                                Commands.parallel(
                                                m_swerveSystem.driveFieldOriented(stationaryAutoAim),
                                                m_shooterSystem.aimAndShootIgnoreCheck(
                                                                () -> m_swerveSystem.getDistanceToTarget(true)),
                                                m_intakeSystem.shuffle(),
                                                m_indexerSystem.run(),
                                                m_intakeSystem.intake()));
        }

        public Command rightNeutralAutoSweepTwice() {
                return Commands.sequence(
                                rightNeutralAutoFirstSweep(),
                                m_autoFactory.trajectoryCmd("RightAuto2a"),
                                m_swerveSystem.stop(),
                                Commands.waitSeconds(8).deadlineFor(
                                                m_swerveSystem.driveFieldOriented(stationaryAutoAim),
                                                m_shooterSystem.aimAndShootIgnoreCheck(
                                                                () -> m_swerveSystem.getDistanceToTarget(true)),
                                                m_intakeSystem.shuffle()),
                                m_autoFactory.trajectoryCmd("RightAuto3a").deadlineFor(
                                                m_shooterSystem.stopShooting(),
                                                m_intakeSystem.extend()),
                                Commands.parallel(
                                                m_swerveSystem.driveFieldOriented(stationaryAutoAim),
                                                m_shooterSystem.aimAndShootIgnoreCheck(
                                                                () -> m_swerveSystem.getDistanceToTarget(true)),
                                                m_indexerSystem.run(),
                                                m_intakeSystem.intake(),
                                                m_intakeSystem.shuffle()));
        }

        public Command rightNeutralAutoThenClimb() {
                return Commands.sequence(
                                rightNeutralAutoFirstSweep(),
                                m_autoFactory.trajectoryCmd("RightAuto2b"),
                                m_swerveSystem.stop(),
                                // TODO: Tune timing of these waits and commands
                                Commands.waitSeconds(4.5).deadlineFor(
                                                m_swerveSystem.driveFieldOriented(stationaryAutoAim),
                                                m_shooterSystem.aimAndShootIgnoreCheck(
                                                                () -> m_swerveSystem.getDistanceToTarget(true))),
                                Commands.waitSeconds(1.0).deadlineFor(
                                                new ConditionalCommand(
                                                                m_swerveSystem.driveFieldOriented(aimTowardsBlue),
                                                                m_swerveSystem.driveFieldOriented(aimTowardsRed),
                                                                m_swerveSystem::isRedAlliance)),
                                m_autoFactory.trajectoryCmd("RightAuto3b").deadlineFor(
                                                m_intakeSystem.stopRoller(),
                                                m_indexerSystem.stop(),
                                                m_elevatorSubsystem.setHeight(ElevatorConstants.SOFT_UPPER_LIMIT),
                                                m_intakeSystem.retract()),
                                m_swerveSystem.stop(),
                                m_elevatorSubsystem.setHeight(ElevatorConstants.SOFT_LOWER_LIMIT));
        }

        public Command leftNeutralAutoThenDepot() {
                return Commands.sequence(
                                leftNeutralAutoFirstSweep(),
                                m_autoFactory.trajectoryCmd("LeftAuto2c"),
                                m_autoFactory.trajectoryCmd("LeftAuto3c").deadlineFor(
                                                m_shooterSystem.aimAndShootIgnoreCheck(
                                                                () -> m_swerveSystem.getDistanceToTarget(true))));
        }

        public Command leftNeutralAutoThenClimb() {
                return Commands.sequence(
                                leftNeutralAutoFirstSweep(),
                                m_autoFactory.trajectoryCmd("LeftAuto2b"),
                                m_swerveSystem.stop(),
                                // TODO: Tune timing of these waits and commands
                                Commands.waitSeconds(4.5).deadlineFor(
                                                m_swerveSystem.driveFieldOriented(stationaryAutoAim),
                                                m_shooterSystem.aimAndShootIgnoreCheck(
                                                                () -> m_swerveSystem.getDistanceToTarget(true))),
                                Commands.waitSeconds(1.0).deadlineFor(
                                                new ConditionalCommand(
                                                                m_swerveSystem.driveFieldOriented(aimTowardsBlue),
                                                                m_swerveSystem.driveFieldOriented(aimTowardsRed),
                                                                m_swerveSystem::isRedAlliance)),
                                m_autoFactory.trajectoryCmd("LeftAuto3b").deadlineFor(
                                                m_intakeSystem.stopRoller(),
                                                m_indexerSystem.stop(),
                                                m_elevatorSubsystem.setHeight(ElevatorConstants.SOFT_UPPER_LIMIT),
                                                m_intakeSystem.retract()),
                                m_swerveSystem.stop(),
                                m_elevatorSubsystem.setHeight(ElevatorConstants.SOFT_LOWER_LIMIT));
        }

        public Command shootPreloadAndClimbLeft() {
                return Commands.sequence(
                                m_autoFactory.resetOdometry("BackUpCenterLeft"),
                                m_autoFactory.trajectoryCmd("BackUpCenterLeft"),
                                Commands.waitSeconds(4.5).deadlineFor(
                                                m_swerveSystem.driveFieldOriented(stationaryAutoAim),
                                                m_shooterSystem.aimAndShootIgnoreCheck(
                                                                () -> m_swerveSystem.getDistanceToTarget(true))),
                                m_autoFactory.trajectoryCmd("CenterToClimbLeft"),
                                m_swerveSystem.stop(),
                                m_autoFactory.trajectoryCmd("LeftAuto3b").deadlineFor(
                                                m_intakeSystem.stopRoller(),
                                                m_indexerSystem.stop(),
                                                m_elevatorSubsystem.setHeight(ElevatorConstants.SOFT_UPPER_LIMIT),
                                                m_intakeSystem.retract()),
                                m_swerveSystem.stop(),
                                m_elevatorSubsystem.setHeight(ElevatorConstants.SOFT_LOWER_LIMIT));
        }

        public Command shootPreloadAndClimbRight() {
                return Commands.sequence(
                                m_autoFactory.resetOdometry("BackUpCenterRight"),
                                m_autoFactory.trajectoryCmd("BackUpCenterRight"),
                                Commands.waitSeconds(4.5).deadlineFor(
                                                m_swerveSystem.driveFieldOriented(stationaryAutoAim),
                                                m_shooterSystem.aimAndShootIgnoreCheck(
                                                                () -> m_swerveSystem.getDistanceToTarget(true))),
                                m_autoFactory.trajectoryCmd("CenterToClimbRight"),
                                m_swerveSystem.stop(),
                                m_autoFactory.trajectoryCmd("RightAuto3b").deadlineFor(
                                                m_intakeSystem.stopRoller(),
                                                m_indexerSystem.stop(),
                                                m_elevatorSubsystem.setHeight(ElevatorConstants.SOFT_UPPER_LIMIT),
                                                m_intakeSystem.retract()),
                                m_swerveSystem.stop(),
                                m_elevatorSubsystem.setHeight(ElevatorConstants.SOFT_LOWER_LIMIT));
        }
}