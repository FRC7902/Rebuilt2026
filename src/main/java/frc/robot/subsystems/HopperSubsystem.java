// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;

public class HopperSubsystem extends SubsystemBase {
  Servo leftServo;
  Servo rightServo;
  /** Creates a new HopperSubsystem. */
  public HopperSubsystem() {
    leftServo = new Servo(HopperConstants.LEFT_SERVO_PWM_ID);
    rightServo = new Servo(HopperConstants.RIGHT_SERVO_PWM_ID);
  }

  public Command expandHopper() {
    return runOnce(() -> {
      leftServo.setAngle(HopperConstants.SERVO_LEFT_OPEN_DEG.in(Degrees));
      rightServo.setAngle(HopperConstants.SERVO_RIGHT_OPEN_DEG.in(Degrees));
    });
  }
  public Command retractHopper(){
    return runOnce(() -> {
      leftServo.setAngle(HopperConstants.SERVO_LEFT_RETRACT_DEG.in(Degrees)); // TODO: Assumes a value of 0 deg would expand the hopper
      rightServo.setAngle(HopperConstants.SERVO_RIGHT_RETRACT_DEG.in(Degrees)); // TODO: Assumes a value of 0 deg would expand the hopper
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
