// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.IntakeConstants;


public class YamsLinearIntakeController {
  private final TalonFX m_linearMotor;
  private final DigitalInput m_fullyRetractedLimitSwitch;
  private final DigitalInput m_fullyExtendedLimitSwitch;
  private final PIDController m_positionPid;

  private double m_estimatedPositionPercent;
  private double m_targetPositionPercent;
  private boolean m_closedLoopEnabled;

  public YamsLinearIntakeController(
      int motorCanId,
      int fullyRetractedSwitchDio,
      int fullyExtendedSwitchDio,
      int currentLimitAmps) {
    m_linearMotor = new TalonFX(motorCanId);
    m_fullyRetractedLimitSwitch = new DigitalInput(fullyRetractedSwitchDio);
    m_fullyExtendedLimitSwitch = new DigitalInput(fullyExtendedSwitchDio);

    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    CurrentLimitsConfigs currentLimits =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(currentLimitAmps)
            .withSupplyCurrentLimitEnable(true);
    motorConfig.CurrentLimits = currentLimits;
    m_linearMotor.getConfigurator().apply(motorConfig);

    m_positionPid =
        new PIDController(
            IntakeConstants.LINEAR_POSITION_KP,
            IntakeConstants.LINEAR_POSITION_KI,
            IntakeConstants.LINEAR_POSITION_KD);
    m_positionPid.setTolerance(IntakeConstants.LINEAR_POSITION_TOLERANCE);

    m_estimatedPositionPercent = 0.0;
    m_targetPositionPercent = 0.0;
    m_closedLoopEnabled = false;
  }

  public void setManualPercent(double percentOutput) {
    m_closedLoopEnabled = false;
    m_linearMotor.set(percentOutput);
  }

  public void setTargetPositionPercent(double targetPositionPercent) {
    m_targetPositionPercent = MathUtil.clamp(targetPositionPercent, 0.0, 1.0);
    m_positionPid.reset();
    m_closedLoopEnabled = true;
  }

  public boolean isAtTargetPosition() {
    return m_positionPid.atSetpoint();
  }

  public double getEstimatedPositionPercent() {
    return m_estimatedPositionPercent;
  }

  public void stop() {
    m_closedLoopEnabled = false;
    m_linearMotor.stopMotor();
  }

  public boolean isFullyRetractedTriggered() {
    // TODO: Verify whether the beam break is active-low before keeping this negation.
    return !m_fullyRetractedLimitSwitch.get();
  }

  public boolean isFullyExtendedTriggered() {
    // TODO: Verify whether the beam break is active-low before keeping this negation.
    return !m_fullyExtendedLimitSwitch.get();
  }

  public void periodic() {
    if (isFullyRetractedTriggered()) {
      m_estimatedPositionPercent = 0.0;
    } else if (isFullyExtendedTriggered()) {
      m_estimatedPositionPercent = 1.0;
    }

    if (!m_closedLoopEnabled) {
      return;
    }

    double output = m_positionPid.calculate(m_estimatedPositionPercent, m_targetPositionPercent);
    output = MathUtil.clamp(output, -IntakeConstants.LINEAR_POSITION_MAX_OUTPUT, IntakeConstants.LINEAR_POSITION_MAX_OUTPUT);

    if (isFullyRetractedTriggered() && output < 0.0) {
      output = 0.0;
    }
    if (isFullyExtendedTriggered() && output > 0.0) {
      output = 0.0;
    }

    m_linearMotor.set(output);
  }
}