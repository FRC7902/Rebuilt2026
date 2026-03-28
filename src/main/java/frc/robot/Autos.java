package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoConstants.Position;
import frc.robot.subsystems.IndexerSystem;
import frc.robot.subsystems.ShooterSystem;
import frc.robot.subsystems.SwerveSystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.subsystems.intake.LinearIntakeSubsystem;

public class Autos {

    private final RobotContainer m_robotContainer;

    private final IndexerSystem m_indexerSystem;
    private final IntakeRollerSubsystem m_intakeRollerSubsystem;
    private final LinearIntakeSubsystem m_linearIntakeSubsystem;
    private final ShooterSystem m_shooterSystem;
    private final SwerveSystem m_swerveSystem;

    public Autos(RobotContainer robotContainer) {
        m_robotContainer = robotContainer;

        m_indexerSystem = robotContainer.m_indexerSystem;
        m_intakeRollerSubsystem = robotContainer.m_intakeRollerSubsystem;
        m_linearIntakeSubsystem = robotContainer.m_linearIntakeSubsystem;
        m_shooterSystem = robotContainer.m_shooterSystem;
        m_swerveSystem = robotContainer.m_swerveSystem;

        if (Constants.TELEMETRY && !DriverStation.isFMSAttached()) {
            StructArrayPublisher<Pose2d> waypointPositions = NetworkTableInstance.getDefault()
                    .getStructArrayTopic("Waypoint Positions", Pose2d.struct)
                    .publish();

            Position[] waypointSelection = {
                    Position.STARTING_LINE_RIGHT,
                    Position.NEUTRAL_RIGHT_1,
                    Position.NEUTRAL_RIGHT_2,
                    Position.NEUTRAL_RIGHT_3,
                    Position.ALLIANCE_RIGHT_1
            };

            waypointPositions.accept(
                    java.util.Arrays.stream(waypointSelection)
                            .map(AutoConstants.positionToPose::get)
                            .toArray(Pose2d[]::new));
        }
    }

    private Command driveToWaypoint(Pose2d waypoint) {
        return new InstantCommand(
                () -> m_swerveSystem.setDriveToWaypoint(waypoint))

                // Wait until the robot is within the specified default tolerances of the
                // waypoint
                .andThen(Commands.waitUntil(
                        m_swerveSystem::isAtWaypoint));
    }

    private Command driveToWaypoint(Pose2d waypoint, Angle angleTolerance) {
        return new InstantCommand(
                () -> m_swerveSystem.setDriveToWaypoint(waypoint))

                // Wait until the robot is within the specified angle tolerance of the waypoint
                .andThen(
                        Commands.waitUntil(
                                () -> m_swerveSystem.isAtWaypoint(
                                        AutoConstants.DEFAULT_WAYPOINT_TOLERANCE,
                                        angleTolerance.in(Degrees))));
    }

    private Command driveToWaypoint(Position position) {
        return driveToWaypoint(AutoConstants.positionToPose.get(position));
    }

    private Command driveToWaypoint(Position position, Angle angleTolerance) {
        return driveToWaypoint(AutoConstants.positionToPose.get(position), angleTolerance);
    }

    private Command resetOdometry(Pose2d waypoint) {
        return Commands.runOnce(() -> m_swerveSystem.resetOdometry(waypoint));
    }

    private Command resetOdometry(Position position) {
        return resetOdometry(AutoConstants.positionToPose.get(position));
    }

    public Command rightNeutralAuto() {

        // TODO: Add alliance flipping util

        return new SequentialCommandGroup(
                resetOdometry(Position.STARTING_LINE_RIGHT),
                new InstantCommand(
                        () -> m_robotContainer.driveAngularVelocity.driveToPoseEnabled(true)),

                driveToWaypoint(Position.NEUTRAL_RIGHT_1),

                // Commands.parallel(
                // m_linearIntakeSubsystem.extend(),
                m_intakeRollerSubsystem.intake(),
                // ),

                // Drive to center of field
                driveToWaypoint(Position.NEUTRAL_RIGHT_2),

                driveToWaypoint(Position.NEUTRAL_RIGHT_3)
                        .deadlineFor(
                                // m_linearIntakeSubsystem.midpoint().andThen(
                                // Commands.parallel(
                                m_intakeRollerSubsystem.stop(),
                                m_indexerSystem.stop()
                        // ))
                        ),

                m_indexerSystem.stop(),

                // Get in shooting position
                driveToWaypoint(Position.ALLIANCE_RIGHT_1),

                // Shoot for 5s
                Commands.deadline(
                        Commands.waitSeconds(4),
                        driveToWaypoint(Position.ALLIANCE_RIGHT_1),
                        m_shooterSystem.aimAndShootIgnoreCheck(
                                m_swerveSystem::getDistanceToTarget),
                        m_indexerSystem.run(),
                        m_intakeRollerSubsystem.intake()
                // m_linearIntakeSubsystem.shuffle()
                ),

                // Drive past trench (close to bump) and extend/run intake
                driveToWaypoint(Position.NEUTRAL_RIGHT_4).deadlineFor(
                        m_shooterSystem.stopShooting(),
                        m_indexerSystem.stop(),
                        m_intakeRollerSubsystem.stop()
                // m_linearIntakeSubsystem.midpoint()
                ),

                driveToWaypoint(Position.NEUTRAL_RIGHT_5).deadlineFor(
                        // m_linearIntakeSubsystem.extend(),
                        m_intakeRollerSubsystem.intake(),
                        m_indexerSystem.run()),

                driveToWaypoint(Position.NEUTRAL_RIGHT_6),

                driveToWaypoint(Position.NEUTRAL_RIGHT_4).deadlineFor(
                        // m_linearIntakeSubsystem.midpoint().andThen(Commands.parallel(
                        m_intakeRollerSubsystem.stop(),
                        m_indexerSystem.stop()
                // ))
                ),

                driveToWaypoint(Position.ALLIANCE_RIGHT_1),

                Commands.parallel(
                        driveToWaypoint(Position.ALLIANCE_RIGHT_1),
                        m_shooterSystem.aimAndShootIgnoreCheck(
                                m_swerveSystem::getDistanceToTarget),
                        m_indexerSystem.run(),
                        m_intakeRollerSubsystem.intake()
                // m_linearIntakeSubsystem.shuffle()
                ));

    }

}
