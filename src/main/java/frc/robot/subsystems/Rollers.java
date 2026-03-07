// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RollerConstants;

public class Rollers extends SubsystemBase {
  /** Creates a new Rollers. */
  private final PWMTalonFX m_motor;

  public Rollers() {
    m_motor = new PWMTalonFX(RollerConstants.ROLLER_MOTOR_PWM_ID);
  }

  public Command intake() {
    return runOnce(() -> m_motor.set(RollerConstants.INTAKE_SPEED));
  }
  public Command outtake() {
    return runOnce(() -> m_motor.set(RollerConstants.OUTAKE_SPEED));
  }
  public Command stopRollers(){
    return runOnce(() -> m_motor.stopMotor());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}