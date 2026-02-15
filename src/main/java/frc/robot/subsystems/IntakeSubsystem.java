// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import yams.gearing.GearBox;

import edu.wpi.first.math.system.plant.DCMotor;
import yams.gearing.MechanismGearing;
import yams.mechanisms.SmartMechanism;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class IntakeSubsystem extends SubsystemBase {
  
  /** Creates a new IntakeSubsystem. */
  private final TalonFX m_intakeMotor;
  private final YamsLinearIntakeController m_linearMotor;
  private final DigitalInput m_fullyRetractedLimitSwitch;
  private final DigitalInput m_fullyExtendedLimitSwitch;

  public IntakeSubsystem() {
    m_fullyRetractedLimitSwitch = new DigitalInput(IntakeConstants.SHALLOW_BUTTON_BREAK_DIO);
    m_fullyExtendedLimitSwitch = new DigitalInput(IntakeConstants.DEEP_BUTTON_BREAK_DIO);
    m_intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_CAN_ID);
    m_linearMotor =
        new YamsLinearIntakeController(
            IntakeConstants.LINEAR_MOTOR_CAN_ID,
            IntakeConstants.SHALLOW_BUTTON_BREAK_DIO,
            IntakeConstants.DEEP_BUTTON_BREAK_DIO,
            IntakeConstants.MOTOR_CURRENT_LIMIT);

    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    CurrentLimitsConfigs currentLimits =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(IntakeConstants.MOTOR_CURRENT_LIMIT)
            .withSupplyCurrentLimitEnable(true);

    motorConfig.CurrentLimits = currentLimits;
    m_intakeMotor.getConfigurator().apply(motorConfig);
  }
  public boolean isFullyRetractedLimitSwitchTriggered() {
    return m_linearMotor.isFullyRetractedTriggered();
  }

  public boolean isFullyExtendedLimitSwitchTriggered() {
    return m_linearMotor.isFullyExtendedTriggered();
  }

  public void stopRoller() {
    m_intakeMotor.stopMotor();
  }

  public void setRollerSpeed(double speed) {
    m_intakeMotor.set(speed);
  }
  
  public void setLinearIntakePercentOutput(double speed) {
    m_linearMotor.setManualPercent(speed);
  }
  
  public void setLinearIntakeTargetPosition(double targetPositionPercent) {
    m_linearMotor.setTargetPositionPercent(targetPositionPercent);
  }

  public boolean isLinearIntakeAtTargetPosition() {
    return m_linearMotor.isAtTargetPosition();
  }

  public double getLinearIntakeEstimatedPosition() {
    return m_linearMotor.getEstimatedPositionPercent();
  }

  public void stopLinearIntake() {
    m_linearMotor.stop();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_linearMotor.periodic();
  }
}