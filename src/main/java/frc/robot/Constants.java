// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;

/** Add your docs here. */
public class Constants {
    public static class HopperConstants {
        public static final int LEFT_SERVO_PWM_ID = 1;
        public static final int RIGHT_SERVO_PWM_ID = 2;
        public static final Angle SERVO_LEFT_OPEN_DEG = Degrees.of(180); // in degrees
        public static final Angle SERVO_RIGHT_OPEN_DEG = Degrees.of(180 - SERVO_LEFT_OPEN_DEG.in(Degrees)); // in degrees
        public static final Angle SERVO_LEFT_RETRACT_DEG = Degrees.of(180 - SERVO_LEFT_OPEN_DEG.in(Degrees)); // in degrees
        public static final Angle SERVO_RIGHT_RETRACT_DEG = Degrees.of(180 - SERVO_RIGHT_OPEN_DEG.in(Degrees));; // in degrees
    }
}
