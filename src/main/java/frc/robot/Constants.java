// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Constants {

    public static class SwerveConstants {
        public static final double MAX_SPEED = Units.feetToMeters(30);

        // Speed scaling factors, should be between 0 and 1
        public static final double MIN_TRANSLATION_SPEED_SCALE = 0.175; // Minimum speed scaling factor for joystick input
        public static final double MIN_ROTATION_SPEED_SCALE = 0.25; // Minimum speed scaling factor for joystick input

        public static final double FAST_DRIVE_RAMP_RATE = 0.15;
    }
public static class PathPlanner {
        public static final double kPDrive = 1.95;
        public static final double kIDrive = 0;
        public static final double kDDrive = 0.01;

        public static final double kPAngle = 2.6;
        public static final double kIAngle = 0;
        public static final double kDAngle = 0.01;

    }
}