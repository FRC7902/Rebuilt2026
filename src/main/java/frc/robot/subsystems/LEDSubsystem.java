// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {

    /**
     * Creates a new LEDSubsystem.
     */
    AddressableLED m_led;
    AddressableLEDBuffer m_ledBuffer;

    boolean hopperEmpty; // TODO: will remove once implemented with shooter subsystem
    boolean autoAlignComplete; // TODO: will remove once implemented with vision subsystem
    boolean isIntaking; // TODO: will remove once implemented with intake
    boolean isShooting; //TODO: will remove once implemented with shooter subsystem

    boolean m_inactiveFirst;
    LEDPattern m_currentPattern;
    LEDPattern m_previousPattern;
    Color m_ledColor;

    double m_SHIFT1;
    double m_SHIFT2;

    double m_SHIFT1START;
    double m_SHIFT2START;

    // Singleton Instance
    static LEDSubsystem ledSubsystem = new LEDSubsystem();

    // TODO: call addperiodic() to change the frequency of periodic
    private LEDSubsystem() {
        m_led = new AddressableLED(Constants.LEDConstants.PWM_PORT);
        m_ledBuffer = new AddressableLEDBuffer(Constants.LEDConstants.LED_LENGTH);
        m_led.setLength(Constants.LEDConstants.LED_LENGTH);
        m_led.start();

        // setting initial
        m_ledColor = Color.kRed;
        m_currentPattern = LEDPattern.solid(m_ledColor);
    }
    public static LEDSubsystem getInstance(){
        return ledSubsystem;
    }
    @Override
    public void periodic() {
    }
    // is this real
    public void teleopInit() {
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            m_inactiveFirst = DriverStation.getGameSpecificMessage().charAt(0) == 'R'; // red is inactive first
        } else {
            m_inactiveFirst = DriverStation.getGameSpecificMessage().charAt(0) == 'B'; // blue is inactive first
        }
        m_SHIFT1 = m_inactiveFirst ? Constants.LEDConstants.SHIFT_2_TIME_SECONDS : Constants.LEDConstants.SHIFT_2_TIME_SECONDS - Constants.LEDConstants.SECONDS_OFFSET;
        m_SHIFT1START = m_inactiveFirst ? Constants.LEDConstants.SHIFT_2_BLINK_START : Constants.LEDConstants.SHIFT_2_BLINK_START - Constants.LEDConstants.SECONDS_OFFSET;
        m_SHIFT2 = m_inactiveFirst ? Constants.LEDConstants.SHIFT_4_TIME_SECONDS : Constants.LEDConstants.SHIFT_4_TIME_SECONDS - Constants.LEDConstants.SECONDS_OFFSET;
        m_SHIFT2START = m_inactiveFirst ? Constants.LEDConstants.SHIFT_4_BLINK_START : Constants.LEDConstants.SHIFT_4_BLINK_START - Constants.LEDConstants.SECONDS_OFFSET;
    }

    public void checkForLEDUpdates() {
        m_previousPattern = m_currentPattern;
        double matchTime = DriverStation.getMatchTime(); // TODO: need to check if this includes auton
        if (autoAlignComplete) {
            m_ledColor = Color.kGreen;
        } else if (isIntaking){
            m_ledColor = Color.kBlue;
        } else if (isShooting){
            m_ledColor = Color.kRed;
        } else {
            m_ledColor = Color.kOrange;
        }
        
        if ((matchTime >= m_SHIFT1 && matchTime < m_SHIFT1START)
            || (matchTime >= m_SHIFT2 && matchTime < m_SHIFT2START)
            || (matchTime >= Constants.LEDConstants.END_GAME_BLINK_START && matchTime < Constants.LEDConstants.END_GAME_TIME_SECONDS)) {
            m_currentPattern = LEDPattern.solid(m_ledColor).blink(Seconds.of(1.5));
        } else {
            m_currentPattern = LEDPattern.solid(m_ledColor);
        }
        if (hopperEmpty) {
            m_currentPattern = m_currentPattern.atBrightness(Percent.of(0.2));
        } else {
            m_currentPattern = m_currentPattern.atBrightness(Percent.of(1));
        }
        // makes sure patterns won't be restarted for no good reason
        if (m_previousPattern != m_currentPattern) {
            // sets the color of the led
            m_led.setData(m_ledBuffer);
            m_currentPattern.applyTo(m_ledBuffer);
        }
    }
}
