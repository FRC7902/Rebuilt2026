// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constant.IntakeConstants;

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
  public final TalonFX m_intakemotor;
  public final TalonFX m_LID;
  private final DigitalInput m_fullyRetractedLimitSwitch;
  private final DigitalInput m_fullyExtendedLimitSwitch;
  private final TalonFXConfiguration m_intakemotorConfig;

public IntakeSubsystem() {
  m_intakemotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_CAN_ID); 
  m_LID = new TalonFX(IntakeConstants.LID_MOTOR_CAN_ID); 
  m_fullyRetractedLimitSwitch = new DigitalInput(IntakeConstants.SHALLOW_BUTTON_BREAK_DIO);
  m_fullyExtendedLimitSwitch = new DigitalInput(IntakeConstants.DEEP_BUTTON_BREAK_DIO);
  
  m_intakemotorConfig = new TalonFXConfiguration();
  CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
    .withSupplyCurrentLimit(IntakeConstants.MOTOR_CURRENT_LIMIT)
    .withSupplyCurrentLimitEnable(true);
  m_intakemotorConfig.CurrentLimits = currentLimits;
  m_intakemotor.getConfigurator().apply(m_intakemotorConfig);
  m_LID.getConfigurator().apply(m_intakemotorConfig);
}

  public boolean m_isfullyRetractedLimitSwitch() {
    return !m_fullyRetractedLimitSwitch.get();
  }

  public boolean m_isfullyExtendedLimitSwitch() {
    return !m_fullyExtendedLimitSwitch.get();
  }

  public void Intake_speed(double speed) {
    m_intakemotor.set(speed);
  }

  public void stop() {
    m_intakemotor.set(0);
    m_intakemotor.stopMotor();
  }

  public void deploy() {
    // If already hit the deep beam break, don't keep driving the motor
    if (m_isfullyExtendedLimitSwitch()) {
      stopDeploy();
      return;
    }
    m_LID.set(IntakeConstants.FULL_SPEED);
  }
  
  public void stopDeploy() {
    m_LID.set(0);
    m_LID.stopMotor();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
