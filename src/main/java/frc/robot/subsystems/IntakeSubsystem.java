// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final TalonFX m_intakeMotor;
  private final TalonFX m_linearMotor;
  private final DigitalInput m_fullyRetractedLimitSwitch;
  private final DigitalInput m_fullyExtendedLimitSwitch;

  public IntakeSubsystem() {
    m_intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_CAN_ID);
    m_linearMotor = new TalonFX(IntakeConstants.LINEAR_MOTOR_CAN_ID);
    m_fullyRetractedLimitSwitch = new DigitalInput(IntakeConstants.SHALLOW_BUTTON_BREAK_DIO);
    m_fullyExtendedLimitSwitch = new DigitalInput(IntakeConstants.DEEP_BUTTON_BREAK_DIO);

    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    CurrentLimitsConfigs currentLimits =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(IntakeConstants.MOTOR_CURRENT_LIMIT)
            .withSupplyCurrentLimitEnable(true);

            motorConfig.CurrentLimits = currentLimits;
            m_intakeMotor.getConfigurator().apply(motorConfig);
            m_linearMotor.getConfigurator().apply(motorConfig);
          }
        
  public boolean isFullyRetractedLimitSwitchTriggered() {
    return !m_fullyRetractedLimitSwitch.get();
  }

  public boolean isFullyExtendedLimitSwitchTriggered() {
    return !m_fullyExtendedLimitSwitch.get();
  }

  public void stopIntake() {
    m_intakeMotor.stopMotor();
  }

  public void setLinearMotorSpeed(double speed) {
    m_linearMotor.set(speed);
  }

  public void stopLinearMotor() {
    m_linearMotor.stopMotor();
  }

  public void setSpeed (double speed) {
    m_intakeMotor.set(speed);
  }
  
  
  public void setLinearIntakePosition(double positionPercent) {
    double clampedPosition = MathUtil.clamp(positionPercent, 0.0, 1.0);
  
    if (clampedPosition >= 1.0) {
      setLinearMotorSpeed(IntakeConstants.LINEAR_INTAKE_DEPLOY_SPEED);
    } else if (clampedPosition <= 0.0) {
      setLinearMotorSpeed(-IntakeConstants.LINEAR_INTAKE_DEPLOY_SPEED);
    } else {
      stopLinearMotor();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
