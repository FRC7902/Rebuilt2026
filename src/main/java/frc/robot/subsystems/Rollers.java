// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RollerConstants;

public class Rollers extends SubsystemBase {
  /** Creates a new Rollers. */
  private final TalonFX m_rollerMotor;

  public Rollers() {
    m_rollerMotor = new TalonFX(RollerConstants.ROLLER_MOTOR_CAN_ID);

    TalonFXConfigurator rollerConfig = m_rollerMotor.getConfigurator();
    CurrentLimitsConfigs limitsConfigs = new CurrentLimitsConfigs();

    limitsConfigs.StatorCurrentLimit = RollerConstants.STATOR_CURRENT_LIMIT;
    limitsConfigs.StatorCurrentLimitEnable = true;

    limitsConfigs.SupplyCurrentLimit = RollerConstants.SUPPLY_CURRENT_LIMIT;
    limitsConfigs.SupplyCurrentLimitEnable = true;

    rollerConfig.apply(limitsConfigs);
  }

  public Command intake() {
    return runOnce(() -> m_rollerMotor.set(RollerConstants.INTAKE_SPEED));
  }
  public Command outtake() {
    return runOnce(() -> m_rollerMotor.set(RollerConstants.OUTAKE_SPEED));
  }
  public Command stopRollers(){
    return runOnce(() -> m_rollerMotor.stopMotor());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}