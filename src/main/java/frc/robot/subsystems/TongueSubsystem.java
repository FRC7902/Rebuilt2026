package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
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
import yams.motorcontrollers.local.SparkWrapper;

public class TongueSubsystem extends SubsystemBase {

    private SparkMax m_spark;
    private SmartMotorControllerConfig m_tongueConfig;
    private SmartMotorController m_tongueMotor;
    private ElevatorConfig m_tongueElevConfig;
    private Elevator m_tongue;

    public TongueSubsystem() {

        m_tongueConfig = new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            // Mechanism Circumference is the distance traveled by each mechanism rotation
            // converting rotations to meters.
            .withMechanismCircumference(Meters.of(Inches.of(0.25).in(Meters) * 22))
            // Feedback Constants (PID Constants)
            .withClosedLoopController(ClimbConstants.TongueConstants.kP, ClimbConstants.TongueConstants.kI, ClimbConstants.TongueConstants.kD, MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5))
            .withSimClosedLoopController(ClimbConstants.TongueConstants.kP, ClimbConstants.TongueConstants.kI, ClimbConstants.TongueConstants.kD, MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5))
            // Feedforward Constants
            .withFeedforward(new ElevatorFeedforward(ClimbConstants.TongueConstants.kS, ClimbConstants.TongueConstants.kG, ClimbConstants.TongueConstants.kV))
            .withSimFeedforward(new ElevatorFeedforward(ClimbConstants.TongueConstants.kS, ClimbConstants.TongueConstants.kG, ClimbConstants.TongueConstants.kV))
            // Telemetry name and verbosity level
            .withTelemetry("TongueMotor", TelemetryVerbosity.HIGH)
            // Gearing from the motor rotor to final shaft.
            // In this example GearBox.fromReductionStages(3,4) is the same as
            // GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to
            // your motor.
            // You could also use .withGearing(12) which does the same thing.
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
            // Motor properties to prevent over currenting.
            .withMotorInverted(false)
            .withIdleMode(MotorMode.BRAKE)
            .withStatorCurrentLimit(Amps.of(ClimbConstants.TongueConstants.STATOR_CURRENT_LIMIT))
            .withClosedLoopRampRate(Seconds.of(0.25))
            .withOpenLoopRampRate(Seconds.of(0.25));

        // Vendor motor controller object
        m_spark = new SparkMax(ClimbConstants.TongueConstants.MOTOR_CAN_ID, MotorType.kBrushless);
        m_tongueMotor = new SparkWrapper(m_spark, DCMotor.getNEO(1), m_tongueConfig);

        m_tongueElevConfig = new ElevatorConfig(m_tongueMotor)
            .withStartingHeight(ClimbConstants.TongueConstants.MIN_LENGTH)
            .withHardLimits(ClimbConstants.TongueConstants.MIN_LENGTH, ClimbConstants.TongueConstants.MAX_LENGTH)
            .withTelemetry("Tongue", TelemetryVerbosity.HIGH)
            .withMass(Pounds.of(ClimbConstants.TongueConstants.MASS_LBS));

        m_tongue = new Elevator(m_tongueElevConfig);
    }

    /**
     * Extend the tongue to max length.
     *
     * @return A Command
     */
    public Command extend() {
        return m_tongue.runTo(ClimbConstants.TongueConstants.MAX_LENGTH, ClimbConstants.TongueConstants.MAX_LENGTH);
    }

    /**
     * Retract the tongue to min length.
     *
     * @return A Command
     */
    public Command retract() {
        return m_tongue.runTo(ClimbConstants.TongueConstants.MIN_LENGTH, ClimbConstants.TongueConstants.MIN_LENGTH);
    }

    @Override
    public void periodic() {
        m_tongue.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        m_tongue.simIterate();
    }
}