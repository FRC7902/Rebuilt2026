package frc.subsystems;

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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ClimbConstants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.remote.TalonFXWrapper;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

/**
 * Climber subsystem for the 2026 FRC game.
 *
 * Controls a two-motor arm used to climb to fixed ladder heights.
 * Current focus is Level 1, with support for Levels 2 and 3.
 */
public class ClimbSubsystem extends SubsystemBase {

  // Main Elevator
  private SmartMotorControllerConfig m_leaderConfig;

  private SmartMotorController m_leaderMotor;

  private SmartMotorControllerConfig m_tongueConfig;

  private ElevatorConfig m_elevconfig;

  private Elevator m_elevator;

  // Vendor motor controller object
  private SparkMax spark;

  // Tongue
  private SmartMotorController m_tongueMotor;

  private ElevatorConfig m_tongueElevConfig;

  private Elevator m_tongue;

  /**
   * Creates the climber subsystem
   */
  public ClimbSubsystem() {

    m_leaderConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        // Mechanism Circumference is the distance traveled by each mechanism rotation
        // converting rotations to meters.
        .withMechanismCircumference(Meters.of(Inches.of(0.25).in(Meters) * 22))
        // Feedback Constants (PID Constants)
        .withClosedLoopController(ClimbConstants.ELEVATOR_kP, ClimbConstants.ELEVATOR_kI, ClimbConstants.ELEVATOR_kD, MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5))
        .withSimClosedLoopController(ClimbConstants.ELEVATOR_kP, ClimbConstants.ELEVATOR_kI, ClimbConstants.ELEVATOR_kD, MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5))
        // Feedforward Constants
        .withFeedforward(new ElevatorFeedforward(ClimbConstants.ELEVATOR_kS, ClimbConstants.ELEVATOR_kG, ClimbConstants.ELEVATOR_kV))
        .withSimFeedforward(new ElevatorFeedforward(ClimbConstants.ELEVATOR_kS, ClimbConstants.ELEVATOR_kG, ClimbConstants.ELEVATOR_kV))
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
        .withStatorCurrentLimit(Amps.of(ClimbConstants.ELEVATOR_STATOR_CURRENT_LIMIT))
        .withClosedLoopRampRate(Seconds.of(0.25))
        .withOpenLoopRampRate(Seconds.of(0.25))
        .withFollowers(Pair.of(new TalonFX(ClimbConstants.CLIMBER_FOLLOWER_MOTOR_CAN_ID), false));

    m_leaderMotor = new TalonFXWrapper(
        new TalonFX(ClimbConstants.CLIMBER_LEADER_MOTOR_CAN_ID), DCMotor.getFalcon500(1), m_leaderConfig);

    m_elevconfig = new ElevatorConfig(m_leaderMotor)
        .withStartingHeight(ClimbConstants.ELEVATOR_STARTING_HEIGHT)
        .withHardLimits(ClimbConstants.ELEVATOR_MIN_HEIGHT, ClimbConstants.MAX_HEIGHT)
        .withTelemetry("Elevator", TelemetryVerbosity.HIGH)
        .withMass(Pounds.of(ClimbConstants.ELEVATOR_MASS_LBS));

    m_elevator = new Elevator(m_elevconfig);

    m_tongueConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        // Mechanism Circumference is the distance traveled by each mechanism rotation
        // converting rotations to meters.
        .withMechanismCircumference(Meters.of(Inches.of(0.25).in(Meters) * 22))
        // Feedback Constants (PID Constants)
        .withClosedLoopController(ClimbConstants.TONGUE_kP, ClimbConstants.TONGUE_kI, ClimbConstants.TONGUE_kD, MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5))
        .withSimClosedLoopController(ClimbConstants.TONGUE_kP, ClimbConstants.TONGUE_kI, ClimbConstants.TONGUE_kD, MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5))
        // Feedforward Constants
        .withFeedforward(new ElevatorFeedforward(ClimbConstants.TONGUE_kS, ClimbConstants.TONGUE_kG, ClimbConstants.TONGUE_kV))
        .withSimFeedforward(new ElevatorFeedforward(ClimbConstants.TONGUE_kS, ClimbConstants.TONGUE_kG, ClimbConstants.TONGUE_kV))
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
        .withStatorCurrentLimit(Amps.of(ClimbConstants.TONGUE_STATOR_CURRENT_LIMIT))
        .withClosedLoopRampRate(Seconds.of(0.25))
        .withOpenLoopRampRate(Seconds.of(0.25));

    // Vendor motor controller object
    spark = new SparkMax(ClimbConstants.TONGUE_MOTOR_CAN_ID, MotorType.kBrushless);
    m_tongueMotor = new SparkWrapper(spark, DCMotor.getNEO(1), m_tongueConfig);

    m_tongueElevConfig = new ElevatorConfig(m_tongueMotor)
        .withStartingHeight(ClimbConstants.MIN_TONGUE_LENGTH)
        .withHardLimits(ClimbConstants.MIN_TONGUE_LENGTH, ClimbConstants.MAX_TONGUE_LENGTH)
        .withTelemetry("Tongue", TelemetryVerbosity.HIGH)
        .withMass(Pounds.of(ClimbConstants.TONGUE_MASS_LBS));

    m_tongue = new Elevator(m_tongueElevConfig);
  }
  /**
   * Set the height of the elevator and does not end the command when reached.
   *
   * @param angle Distance to go to.
   * @return a Command
   */
  public Command setHeight(Distance height) {
    return m_elevator.run(height);
  }

  /**
   * Set the height of the elevator and ends the command when reached, but not the
   * closed loop controller.
   *
   * @param angle Distance to go to.
   * @return A Command
   */
  public Command setHeightAndStop(Distance height) {
    return m_elevator.runTo(height, height);
  }

  /**
   * Set the elevators closed loop controller setpoint.
   *
   * @param angle Distance to go to.
   */
  public void setHeightSetpoint(Distance height) {
    m_elevator.setMeasurementPositionSetpoint(height);
  }

  /**
   * Move the elevator up and down.
   *
   * @param dutycycle [-1, 1] speed to set the elevator too.
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

  public Command extendTongue() {
    return m_tongue.runTo(ClimbConstants.MAX_TONGUE_LENGTH, ClimbConstants.MAX_TONGUE_LENGTH);
  }

  public Command retractTongue() {
    return m_tongue.runTo(ClimbConstants.MIN_TONGUE_LENGTH, ClimbConstants.MIN_TONGUE_LENGTH);
  }

  /**
   * 1. Elevator starts at 0m height
   * 2. Bring elevator to max height to grab the first (lowest) rung
   * 3. Bring elevator down a few inches to S3 (L1 is complete)
   */
  public Command climbL1() {
    return new SequentialCommandGroup(
        setHeightAndStop(ClimbConstants.MAX_HEIGHT),
        setHeightAndStop(ClimbConstants.SETPOINT_3)
    );
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
        setHeightAndStop(ClimbConstants.SETPOINT_1),
        setHeightAndStop(ClimbConstants.MAX_HEIGHT),
        extendTongue(),
        setHeightAndStop(ClimbConstants.SETPOINT_2),
        retractTongue()
    );
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
        setHeightAndStop(ClimbConstants.SETPOINT_1),
        setHeightAndStop(ClimbConstants.MAX_HEIGHT),
        extendTongue(),
        setHeightAndStop(ClimbConstants.SETPOINT_2),
        retractTongue()
    );
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_elevator.updateTelemetry();
    m_tongue.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    m_elevator.simIterate();
    m_tongue.simIterate();
  }
}
