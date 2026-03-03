// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.thethriftybot.devices.ThriftyNova;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import yams.motorcontrollers.local.NovaWrapper;
import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class TongueSubsystem extends SubsystemBase {
    // Vendor motor controller object
    private ThriftyNova m_tongueMotor = new ThriftyNova(ClimbConstants.TongueConstants.MOTOR_CAN_ID);

    // Motor configs for both elevator motors (they do the same thing)
    private SmartMotorControllerConfig tongueSmartControllerConfig = new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            // Mechanism Circumference is the distance traveled by each mechanism rotation
            // converting rotations to meters.
            .withMechanismCircumference(Meters.of(Inches.of(0.25).in(Meters) * 22))
            // Feedback Constants (PID Constants)
            .withClosedLoopController(ClimbConstants.TongueConstants.kP, ClimbConstants.TongueConstants.kI, ClimbConstants.TongueConstants.kD,
                    MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5))
            .withSimClosedLoopController(ClimbConstants.TongueConstants.kP, ClimbConstants.TongueConstants.kI, ClimbConstants.TongueConstants.kD,
                    MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5))
            // Feedforward Constants
            .withFeedforward(new ElevatorFeedforward(0, 0, 0))
            .withSimFeedforward(new ElevatorFeedforward(0, 0, 0))
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
            .withStatorCurrentLimit(Amps.of(40))
            .withClosedLoopRampRate(Seconds.of(0.25));

    // SmartMotorController using TalonFX Wrapper
    private SmartMotorController tongueMotorController = new NovaWrapper(m_tongueMotor, DCMotor.getNeo550(1),
            tongueSmartControllerConfig);

    private ElevatorConfig tongueConfig = new ElevatorConfig(tongueMotorController)
            .withStartingHeight(Meters.of(0.5))
            .withHardLimits(Meters.of(0), Meters.of(3))
            .withTelemetry("Tongue", TelemetryVerbosity.HIGH)
            .withMass(Pounds.of(16))
            .withAngle(Degrees.of(0));
    // Elevator Mechanism
    private Elevator tongue = new Elevator(tongueConfig);

    // private Elevator tongue = new Elevator(tongConfig);
    /**
     * Set the height of the elevator and does not end the command when reached.
     * 
     * @param angle Distance to go to.
     * @return a Command
     */
    public Command setLength(Distance length) {
        return tongue.runTo(length, Meters.of(0.01));
    }

    /**
     * Set the elevators closed loop controller setpoint.
     * 
     * @param angle Distance to go to.
     */
    public void setLengthSetpoint(Distance length) {
        tongue.setMeasurementPositionSetpoint(length);
    }

    /**
     * Move the elevator up and down.
     * 
     * @param dutycycle [-1, 1] speed to set the elevator too.
     */
    public Command set(double dutycycle) {
        return tongue.set(dutycycle);
    }

    /**
     * Run sysId on the {@link Elevator}
     */
    public Command sysId() {
        return tongue.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));
    }

    /** Creates a new Climber. */
    public TongueSubsystem() {
        // m_elevatorFollowerMotor.setControl(new
        // Follower(ClimbConstants.LEADER_MOTOR_CAN_ID, MotorAlignmentValue.Opposed));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        tongue.updateTelemetry();
        // System.out.println("Tongue Compare: " +
        // m_lengthSetpoint.minus(tongue.getHeight()));
        // System.out.println("Tongue Setpoint: " + m_lengthSetpoint);
        // System.out.println("Current Lenght: " + tongue.getHeight());
        // System.out.println("Tongue: " + isAtTargetLength());

    }

    public void simulationPeriodic() {
        tongue.simIterate();
    }
}