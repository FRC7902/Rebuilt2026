// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  AddressableLED m_led;
  AddressableLEDBuffer m_ledBuffer;
  public LEDSubsystem() {
    m_led = new AddressableLED(Constants.LEDConstants.PWM_PORT);
    m_ledBuffer = new AddressableLEDBuffer(Constants.LEDConstants.LED_LENGTH);
    m_led.setLength(Constants.LEDConstants.LED_LENGTH);
    m_led.start();
  }

  @Override
  public void periodic() {
    m_led.setData(m_ledBuffer);
  }


}
