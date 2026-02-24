// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RollerConstants;

public class RollerSubsystem extends SubsystemBase {
  private final TalonFX m_rollerMotor;
  /** Creates a new RollerSubsystem. */
  public RollerSubsystem() {
    m_rollerMotor = new TalonFX(RollerConstants.ROLLER_MOTOR_CAN_ID);
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
