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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  AddressableLED m_led;
  AddressableLEDBuffer m_ledBuffer;

  LEDPattern hubTransitionPattern;
  boolean hopperEmpty; // TODO: will remove once implemented with shooter subsystem
  boolean hubTransition;
  Trigger hubTrigger;
  Color ledColor;

  ArrayList<Double> TransitionTimes;

  public LEDSubsystem() {
    m_led = new AddressableLED(Constants.LEDConstants.PWM_PORT);
    m_ledBuffer = new AddressableLEDBuffer(Constants.LEDConstants.LED_LENGTH);
    m_led.setLength(Constants.LEDConstants.LED_LENGTH);
    m_led.start();

    // setting up patterns
    hubTransitionPattern = LEDPattern.solid(ledColor).blink(Seconds.of(1.5));
    // transition times between alliance shifts
    TransitionTimes = new ArrayList<>();
    TransitionTimes.addAll(Arrays.asList(55.0,80.0,105.0,130.0)); // in seconds

    // setting up triggers
    hubTrigger = new Trigger(() -> hubTransition);
    ledColor = Color.kRed;

    setDefaultCommand(runPattern(LEDPattern.solid(ledColor))); //when not empty
  }

  @Override
  public void periodic() {
    setHubTransition();
    // TODO: set hopperEmpty to the value from the shooter subsystem
    ledColor = hopperEmpty ? Color.kGreen : Color.kRed;
    hubTransitionPattern = LEDPattern.solid(ledColor).blink(Seconds.of(1.5));
    // sets the color of the led
    m_led.setData(m_ledBuffer);
  }
  public Trigger getHubTrigger(){
    return hubTrigger;
  }
  public LEDPattern getHubPattern(){
    return hubTransitionPattern;
  }
  public void setHubTransition() {
    double matchTime = DriverStation.getMatchTime(); // need to check if this includes auton
    
    for (Double i : TransitionTimes){
      if (i - matchTime > 0 && i - matchTime <= Constants.LEDConstants.shiftInterval){
        hubTransition = true;
        return;
      }
    }
    hubTransition = false;
  }

  public Command runPattern(LEDPattern pattern){
    return run(() -> pattern.applyTo(m_ledBuffer));
  }
}
