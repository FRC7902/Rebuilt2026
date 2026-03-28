// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.thethriftybot.devices.ThriftyNova;
import com.thethriftybot.devices.ThriftyNova.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants.TongueConstants;
import frc.robot.Robot;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.NovaWrapper;

public class TongueSubsystem extends SubsystemBase {

    private final ThriftyNova m_motor;
    private final SmartMotorController m_smartMotorController;
    private final Elevator m_tongue;

    public TongueSubsystem() {
        m_motor = new ThriftyNova(TongueConstants.MOTOR_CAN_ID, MotorType.NEO);

        SmartMotorControllerConfig motorConfig = TongueConstants.SMC_CONFIG
                .apply(new SmartMotorControllerConfig(this));

        m_smartMotorController = new NovaWrapper(m_motor, DCMotor.getNeo550(1), motorConfig);

        ElevatorConfig tongueConfig = TongueConstants.ELEVATOR_CONFIG
                .apply(new ElevatorConfig(m_smartMotorController));

        if (Robot.isSimulation()) {
            tongueConfig.withStartingHeight(TongueConstants.STARTING_HEIGHT);
        }

        m_tongue = new Elevator(tongueConfig);
    }

    /**
     * Creates a SysId characterization command for the tongue.
     *
     * @return the SysId command
     */
    public Command sysId() {
        return m_tongue.sysId(
                        Volts.of(12), Volts.of(12).per(Second), Second.of(30))
                .beforeStarting(
                        () -> SignalLogger.start())
                .finallyDo(() -> SignalLogger.stop());
    }

    public Command elevCmd(double dutycycle) {
        return m_tongue.set(dutycycle);
    }

    public Command setDistance(Distance distance) {
        return m_tongue.setHeight(distance);
    }

    public Command stop() {
        return setDistance(m_tongue.getHeight());
    }

    @Override
    public void periodic() {
        m_tongue.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        m_tongue.simIterate();
    }
}
