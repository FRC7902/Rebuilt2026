package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ClimbConstants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class ElevatorSubsystem extends SubsystemBase {

    private SmartMotorControllerConfig m_leaderConfig;
    private SmartMotorController m_leaderMotor;
    private ElevatorConfig m_elevConfig;
    private Elevator m_elevator;

    public ElevatorSubsystem() {

        m_leaderConfig = new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            // Mechanism Circumference is the distance traveled by each mechanism rotation
            // converting rotations to meters.
            .withMechanismCircumference(Meters.of(Inches.of(0.25).in(Meters) * 22))
            // Feedback Constants (PID Constants)
            .withClosedLoopController(ClimbConstants.ElevatorConstants.kP, ClimbConstants.ElevatorConstants.kI, ClimbConstants.ElevatorConstants.kD, MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5))
            .withSimClosedLoopController(ClimbConstants.ElevatorConstants.kP, ClimbConstants.ElevatorConstants.kI, ClimbConstants.ElevatorConstants.kD, MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5))
            // Feedforward Constants
            .withFeedforward(new ElevatorFeedforward(ClimbConstants.ElevatorConstants.kS, ClimbConstants.ElevatorConstants.kG, ClimbConstants.ElevatorConstants.kV))
            .withSimFeedforward(new ElevatorFeedforward(ClimbConstants.ElevatorConstants.kS, ClimbConstants.ElevatorConstants.kG, ClimbConstants.ElevatorConstants.kV))
            // Telemetry name and verbosity level
            .withTelemetry("ClimbLeader", TelemetryVerbosity.HIGH)
            // Gearing from the motor rotor to final shaft.
            // In this example GearBox.fromReductionStages(3,4) is the same as
            // GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to
            // your motor.
            // You could also use .withGearing(12) which does the same thing
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
            // Motor properties to prevent over currenting.
            .withMotorInverted(false)
            .withIdleMode(MotorMode.BRAKE)
            .withStatorCurrentLimit(Amps.of(ClimbConstants.ElevatorConstants.STATOR_CURRENT_LIMIT))
            .withClosedLoopRampRate(Seconds.of(0.25))
            .withOpenLoopRampRate(Seconds.of(0.25))
            .withFollowers(Pair.of(new TalonFX(ClimbConstants.ElevatorConstants.FOLLOWER_MOTOR_CAN_ID), true));

        m_leaderMotor = new TalonFXWrapper(
            new TalonFX(ClimbConstants.ElevatorConstants.LEADER_MOTOR_CAN_ID), DCMotor.getFalcon500(2), m_leaderConfig);

        m_elevConfig = new ElevatorConfig(m_leaderMotor)
            .withStartingHeight(ClimbConstants.ElevatorConstants.STARTING_HEIGHT)
            .withHardLimits(ClimbConstants.ElevatorConstants.MIN_HEIGHT, ClimbConstants.ElevatorConstants.MAX_HEIGHT)
            .withTelemetry("Elevator", TelemetryVerbosity.HIGH)
            .withMass(Pounds.of(ClimbConstants.ElevatorConstants.MASS_LBS));

        m_elevator = new Elevator(m_elevConfig);
    }

    /**
     * Set the height of the elevator and does not end the command when reached.
     *
     * @param height Distance to go to.
     * @return a Command
     */
    public Command setHeight(Distance height) {
        return m_elevator.run(height);
    }

    /**
     * Set the height of the elevator and ends the command when reached, but not the
     * closed loop controller.
     *
     * @param height Distance to go to.
     * @return A Command
     */
    public Command setHeightAndStop(Distance height) {
        return m_elevator.runTo(height, ClimbConstants.ElevatorConstants.TOLERANCE);
    }

    /**
     * Set the elevators closed loop controller setpoint.
     *
     * @param height Distance to go to.
     */
    public void setHeightSetpoint(Distance height) {
        m_elevator.setMeasurementPositionSetpoint(height);
    }

    /**
     * Move the elevator up and down.
     *
     * @param dutycycle [-1, 1] speed to set the elevator to.
     */
    public Command set(double dutycycle) {
        return m_elevator.set(dutycycle);
    }

    /**
     * Run sysId on the {@link Elevator}
     */
    public Command sysId() {
        return m_elevator.sysId(
            Volts.of(7),
            Volts.of(2).per(Second),
            Seconds.of(4));
    }

    @Override
    public void periodic() {
        m_elevator.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        m_elevator.simIterate();
    }
}