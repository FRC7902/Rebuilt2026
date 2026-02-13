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
    static AddressableLED m_led;
   static  AddressableLEDBuffer m_ledBuffer;

    static boolean hopperEmpty; // TODO: will remove once implemented with shooter subsystem
    static boolean autoAlignComplete; // TODO: will remove once implemented with vision subsystem
    static boolean inactiveFirst;
    static boolean isIntakingNotOuttaking; // TODO: will remove once implemented with intake

    static LEDPattern currentPattern;
    static LEDPattern previousPattern;
    static Color ledColor;

    static double SHIFT1;
    static double SHIFT2;

    static double SHIFT1START;
    static double SHIFT2START;

    // TODO: call addperiodic() to change the frequency of periodic
    public LEDSubsystem() {
        m_led = new AddressableLED(Constants.LEDConstants.PWM_PORT);
        m_ledBuffer = new AddressableLEDBuffer(Constants.LEDConstants.LED_LENGTH);
        m_led.setLength(Constants.LEDConstants.LED_LENGTH);
        m_led.start();

        // setting initial
        ledColor = Color.kRed;
        currentPattern = LEDPattern.solid(ledColor);
    }

    @Override
    public void periodic() {
    }
    // is this real
    public static void teleopInit() {
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            inactiveFirst = DriverStation.getGameSpecificMessage().charAt(0) == 'R'; // red is inactive first
        } else {
            inactiveFirst = DriverStation.getGameSpecificMessage().charAt(0) == 'B'; // blue is inactive first
        }
        SHIFT1 = inactiveFirst ? Constants.LEDConstants.SHIFT_2_TIME_SECONDS : Constants.LEDConstants.SHIFT_2_TIME_SECONDS - Constants.LEDConstants.SECONDS_OFFSET;
        SHIFT1START = inactiveFirst ? Constants.LEDConstants.SHIFT_2_BLINK_START : Constants.LEDConstants.SHIFT_2_BLINK_START - Constants.LEDConstants.SECONDS_OFFSET;
        SHIFT2 = inactiveFirst ? Constants.LEDConstants.SHIFT_4_TIME_SECONDS : Constants.LEDConstants.SHIFT_4_TIME_SECONDS - Constants.LEDConstants.SECONDS_OFFSET;
        SHIFT2START = inactiveFirst ? Constants.LEDConstants.SHIFT_4_BLINK_START : Constants.LEDConstants.SHIFT_4_BLINK_START - Constants.LEDConstants.SECONDS_OFFSET;
    }

    public static void checkForLEDUpdates() {
        previousPattern = currentPattern;
        double matchTime = DriverStation.getMatchTime(); // TODO: need to check if this includes auton
        if (autoAlignComplete) {
            ledColor = Color.kRed;
        } else {
            ledColor = Color.kGreen;
        }
        if (isIntakingNotOuttaking) {
            ledColor = Color.kBlue;
        } else {
            ledColor = Color.kOrange;
        }

        if ((matchTime >= SHIFT1 && matchTime < SHIFT1START)
            || (matchTime >= SHIFT2 && matchTime < SHIFT2START)
            || (matchTime >= Constants.LEDConstants.END_GAME_BLINK_START && matchTime < Constants.LEDConstants.END_GAME_TIME_SECONDS)) {
            currentPattern = LEDPattern.solid(ledColor).blink(Seconds.of(1.5));
        } else {
            currentPattern = LEDPattern.solid(ledColor);
        }
        if (hopperEmpty) {
            currentPattern = currentPattern.atBrightness(Percent.of(0.5));
        } else {
            currentPattern = currentPattern.atBrightness(Percent.of(1));
        }
        // makes sure patterns won't be restarted for no good reason
        if (previousPattern != currentPattern) {
            // sets the color of the led
            m_led.setData(m_ledBuffer);
            currentPattern.applyTo(m_ledBuffer);
        }
    }
}
