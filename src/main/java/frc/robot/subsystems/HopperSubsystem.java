// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;

public class HopperSubsystem extends SubsystemBase {
  Servo leftServo;
  Servo rightServo;
  /** Creates a new HopperSubsystem. */
  public HopperSubsystem() {
    leftServo = new Servo(HopperConstants.LEFT_SERVO);
    rightServo = new Servo(HopperConstants.RIGHT_SERVO);
  }

  public void expandHopper() {
    leftServo.setAngle(HopperConstants.HOPPER_EXPANSION_VALUE); // TODO: Assumes a value of 180 deg would expand the hopper
    rightServo.setAngle(HopperConstants.HOPPER_EXPANSION_VALUE); // TODO: Assumes a value of 180 deg would expand the hopper
  }
  public void shrinkHopper(){
    leftServo.setAngle(HopperConstants.HOPPER_SHRINK_VALUE); // TODO: Assumes a value of 0 deg would expand the hopper
    rightServo.setAngle(HopperConstants.HOPPER_SHRINK_VALUE); // TODO: Assumes a value of 0 deg would expand the hopper
  }

  public Command expand(){
    return new InstantCommand(() -> expandHopper());
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
