// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Constants {
    public static class LEDConstants {
        public static final int PWM_PORT = -1; //TODO: Change constants
        public static final int LED_LENGTH = -1;
        public static final int SHIFT_INTERVAL = 5; // in secs

        // inactive first
        public static final double SHIFT_2_TIME_SECONDS = 55.0;
        public static final double SHIFT_2_BLINK_START = SHIFT_2_TIME_SECONDS - SHIFT_INTERVAL;
        public static final double SHIFT_4_TIME_SECONDS = 105.0;
        public static final double SHIFT_4_BLINK_START = SHIFT_4_TIME_SECONDS - SHIFT_INTERVAL;
        public static final double END_GAME_TIME_SECONDS = 130.0;
        public static final double END_GAME_BLINK_START = END_GAME_TIME_SECONDS - SHIFT_INTERVAL * 2;

        public static final double SECONDS_OFFSET = 25;
    }
}
