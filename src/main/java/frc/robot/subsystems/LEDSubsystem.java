// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;

import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  AddressableLED m_led;
  AddressableLEDBuffer m_ledBuffer;

  LEDPattern hubTransitionPattern;

  boolean hopperEmpty;
  boolean hubTransition;

  ArrayList<Double> TransitionTimes;


  public LEDSubsystem() {
    m_led = new AddressableLED(Constants.LEDConstants.PWM_PORT);
    m_ledBuffer = new AddressableLEDBuffer(Constants.LEDConstants.LED_LENGTH);
    m_led.setLength(Constants.LEDConstants.LED_LENGTH);
    m_led.start();

    // setting up all patterns
    String fmsMsg = DriverStation.getGameSpecificMessage();
    DriverStation.getMatchTime();
    if (fmsMsg.length() > 0){
      switch (fmsMsg.charAt(0)){
        case 'B':
          hubTransitionPattern = LEDPattern.solid(Color.kBlue).blink(Seconds.of(1.5));
        case 'R':
          hubTransitionPattern = LEDPattern.solid(Color.kRed).blink(Seconds.of(1.5));
        default:
          hubTransitionPattern = null;
      }
    }
    TransitionTimes = new ArrayList<>();
    TransitionTimes.addAll(Arrays.asList(55.0,80.0,105.0,130.0)); // in seconds

    setDefaultCommand(runPattern(LEDPattern.kOff));
  }

  @Override
  public void periodic() {
    setHubTransition();
    setPattern();
    // sets the color of the led
    m_led.setData(m_ledBuffer);
  }

  public void setHubTransition() {
    double matchTime = DriverStation.getMatchTime();
    for (Double i : TransitionTimes){
      if (i - matchTime > 0 && i - matchTime <= 5){
        hubTransition = true;
        return;
      }
    }
    hubTransition = false;
  }
  public void setHopperEmpty() {
    // need method from feeder subsystem using beam break 2
    // use debouncer
  }
  
  public void setPattern() {
    if (hubTransition) {
      runPattern(hubTransitionPattern);
      return;
    }
    if (hopperEmpty){
      runPattern(LEDPattern.solid(Color.kGreen)); // green when hopper is emptied
    } else {
      runPattern(LEDPattern.solid(Color.kRed)); // red when driver needs to collect more fuel
    }
  }

  public Command runPattern(LEDPattern pattern){
    return run(() -> pattern.applyTo(m_ledBuffer));
  }

}
