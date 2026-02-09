// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Constant {
    public static class IntakeConstants {
        public static final int INTAKE_MOTOR_CAN_ID = 34;
        public static final int LID_MOTOR_CAN_ID = 33;

        public static final double FULL_SPEED = 0.5;
        public static final double OUTTAKE_SPEED = 0.25;
        public static final double STOP_SPEED = 0.0;

        public static final int SHALLOW_BUTTON_BREAK_DIO = 2;
        public static final int DEEP_BUTTON_BREAK_DIO = 3;

        public static final int MOTOR_CURRENT_LIMIT = 20;
    }
    public static class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
        public static final double DEADBAND = 0.15;
    }
}
