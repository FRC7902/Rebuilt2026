package frc.robot.subsystems;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ClimbConstants;

/**
 * Climber subsystem for the 2026 FRC game.
 *
 * Controls a two-motor arm used to climb to fixed ladder heights.
 * Current focus is Level 1, with support for Levels 2 and 3.
 */
public class ClimbSubsystem extends SubsystemBase {

    private final ElevatorSubsystem m_elevator;
    private final TongueSubsystem m_tongue;

    public ClimbSubsystem() {
        m_elevator = new ElevatorSubsystem();
        m_tongue = new TongueSubsystem();
    }

    // Test Commands
    public Command setElevatorHeight(Distance k) {
        return m_elevator.setHeight(k);
    }


    public Command setTongueLength(Distance k) {
        return m_tongue.setLength(k);
    }

    public Command runTongueTest() {
        return new SequentialCommandGroup(
                m_tongue.setLength(ClimbConstants.TongueConstants.MAX_LENGTH)
        );
    }

    /**
     * 1. Elevator starts at 0m height
     * 2. Bring elevator to max height to grab the first (lowest) rung
     * 3. Bring elevator down a few inches to S3 (L1 is complete)
     */
    public Command climbL1() {
        return new SequentialCommandGroup(
                m_elevator.setHeightAndStop(ClimbConstants.ElevatorConstants.MAX_HEIGHT),
                m_elevator.setHeightAndStop(ClimbConstants.ElevatorConstants.SETPOINT_3));
    }

    /**
     * 4. Bring elevator down to S1
     * 5. Bring elevator up to max height to grab the second (middle) rung
     * 6. Extend the tongue out
     * 7. Bring elevator down to S2
     * 8. Retract the tongue (L2 is complete)
     */
    public Command climbL2() {
        return new SequentialCommandGroup(
                climbL1(),
                m_elevator.setHeightAndStop(ClimbConstants.ElevatorConstants.SETPOINT_1),
                m_elevator.setHeightAndStop(ClimbConstants.ElevatorConstants.MAX_HEIGHT),
                m_tongue.setLength(ClimbConstants.TongueConstants.MAX_LENGTH),
                m_elevator.setHeightAndStop(ClimbConstants.ElevatorConstants.SETPOINT_2),
                m_tongue.setLength(ClimbConstants.TongueConstants.MIN_LENGTH));
    }

    /**
     * 9. Bring elevator down to S1
     * 10. Bring elevator to max height to grab the third (topmost) rung
     * 11. Extend the tongue out
     * 12. Bring elevator down to S2
     * 13. Retract the tongue (L3 is complete)
     */
    public Command climbL3() {
        return new SequentialCommandGroup(
                climbL2(),
                m_elevator.setHeightAndStop(ClimbConstants.ElevatorConstants.SETPOINT_1),
                m_elevator.setHeightAndStop(ClimbConstants.ElevatorConstants.MAX_HEIGHT),
                m_tongue.setLength(ClimbConstants.TongueConstants.MAX_LENGTH),
                m_elevator.setHeightAndStop(ClimbConstants.ElevatorConstants.SETPOINT_2),
                m_tongue.setLength(ClimbConstants.TongueConstants.MIN_LENGTH));
    }

    /**
     * Reverse L1: Bring elevator back up to max, then back to starting position
     */
    public Command reverseL1() {
        return new SequentialCommandGroup(
                m_elevator.setHeightAndStop(ClimbConstants.ElevatorConstants.MAX_HEIGHT),
                m_elevator.setHeightAndStop(ClimbConstants.ElevatorConstants.MIN_HEIGHT));
    }

    /**
     * Reverse L2: Undo L2 steps in reverse order
     * 1. Extend tongue
     * 2. Bring elevator back up to max height
     * 3. Retract tongue
     * 4. Bring elevator down to S1
     * 5. Then reverse L1
     */
    public Command reverseL2() {
        return new SequentialCommandGroup(
                m_tongue.setLength(ClimbConstants.TongueConstants.MAX_LENGTH),
                m_elevator.setHeightAndStop(ClimbConstants.ElevatorConstants.MAX_HEIGHT),
                m_tongue.setLength(ClimbConstants.TongueConstants.MIN_LENGTH),
                m_elevator.setHeightAndStop(ClimbConstants.ElevatorConstants.SETPOINT_1),
                reverseL1());
    }

    /**
     * Reverse L3: Undo L3 steps in reverse order
     * 1. Extend tongue
     * 2. Bring elevator back up to max height
     * 3. Retract tongue
     * 4. Bring elevator down to S1
     * 5. Then reverse L2
     */
    public Command reverseL3() {
        return new SequentialCommandGroup(
                m_tongue.setLength(ClimbConstants.TongueConstants.MAX_LENGTH),
                m_elevator.setHeightAndStop(ClimbConstants.ElevatorConstants.MAX_HEIGHT),
                m_tongue.setLength(ClimbConstants.TongueConstants.MIN_LENGTH),
                m_elevator.setHeightAndStop(ClimbConstants.ElevatorConstants.SETPOINT_1),
                reverseL2());
    }
}