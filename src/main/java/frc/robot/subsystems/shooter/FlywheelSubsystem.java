// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterConstants.FlywheelConstants;

public class FlywheelSubsystem extends SubsystemBase {

    private final TalonFX m_leaderMotor = new TalonFX(FlywheelConstants.LEADER_MOTOR_CAN_ID);;
    private final TalonFX m_followerMotor = new TalonFX(FlywheelConstants.FOLLOWER_MOTOR_CAN_ID);

    private final DutyCycleOut m_dutyCycle = new DutyCycleOut(0);
    private final VelocityTorqueCurrentFOC m_velocityTorque = new VelocityTorqueCurrentFOC(0).withSlot(0);
    private final NeutralOut m_brake = new NeutralOut();

    private final VoltageOut m_sysIdControl = new VoltageOut(0);
    private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(12), // Reduce dynamic voltage to 4 to prevent brownout
                    null, // Use default timeout (10 s)
                          // Log state with Phoenix SignalLogger class
                    state -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    volts -> m_leaderMotor.setControl(m_sysIdControl.withOutput(volts)),
                    null,
                    this));

    public FlywheelSubsystem() {
        TalonFXConfiguration configs = new TalonFXConfiguration();

        // Only need kP and kS when using TorqueCurrentFOC

        configs.Slot0.kP = FlywheelConstants.PID_kP;
        configs.Slot0.kS = FlywheelConstants.FF_kS;

        configs.TorqueCurrent.withPeakForwardTorqueCurrent(FlywheelConstants.STATOR_CURRENT_LIMIT_AMPS)
                .withPeakReverseTorqueCurrent(Amps.of(FlywheelConstants.STATOR_CURRENT_LIMIT_AMPS.in(Amps) * -1));

        /* Retry config apply up to 5 times, report if failure */
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = m_leaderMotor.getConfigurator().apply(configs);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }

        m_followerMotor.setControl(new Follower(m_leaderMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    /**
     * Creates a command to set the flywheel motor output as a percentage of maximum
     * voltage.
     * 
     * @param dutyCycle the percentage of maximum voltage to apply to the flywheel
     *                  motors (between -1.0 and 1.0)
     * @return the command that sets the flywheel motor output to the specified duty
     *         cycle
     */
    public Command setDutyCycle(double dutyCycle) {
        return new InstantCommand(() -> m_leaderMotor.setControl(m_dutyCycle.withOutput(dutyCycle)));
    }

    /**
     * Creates a command to set the flywheel velocity.
     *
     * @param speed the target angular velocity
     * @return the command that sets flywheel speed
     */
    public Command setSpeed(AngularVelocity speed) {
        return new InstantCommand(
                () -> m_leaderMotor.setControl(m_velocityTorque.withVelocity(speed)));
    }

    /**
     * Creates a command to set the flywheel velocity from a supplier.
     *
     * @param speed the supplier of target angular velocity
     * @return the command that sets flywheel speed
     */
    public Command setSpeed(Supplier<AngularVelocity> speed) {
        return new InstantCommand(() -> m_leaderMotor.setControl(m_velocityTorque.withVelocity(speed.get())));
    }

    /**
     * Creates a command to stop the flywheel by setting the motor output to zero.
     * 
     * @return the command that stops the flywheel
     */
    public Command stop() {
        return new InstantCommand(() -> m_leaderMotor.setControl(m_brake));
    }

    /**
     * Creates a SysId characterization command for the flywheel.
     *
     * @return the SysId command
     */
    public Command sysId() {
        return Commands.sequence(
                m_sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward),
                m_sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
                m_sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward),
                m_sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse))
                .beforeStarting(
                        new InstantCommand(() -> {
                            BaseStatusSignal.setUpdateFrequencyForAll(250,
                                    m_leaderMotor.getPosition(),
                                    m_leaderMotor.getVelocity(),
                                    m_leaderMotor.getMotorVoltage());

                            m_leaderMotor.optimizeBusUtilization();

                            SignalLogger.start();
                        }))
                .finallyDo(
                        () -> {
                            SignalLogger.stop();
                            BaseStatusSignal.setUpdateFrequencyForAll(50,
                                    m_leaderMotor.getPosition(),
                                    m_leaderMotor.getVelocity(),
                                    m_leaderMotor.getMotorVoltage());

                            m_leaderMotor.resetSignalFrequencies();
                        });
    }

    /**
     * Gets the current flywheel velocity.
     *
     * @return the current angular velocity
     */
    public AngularVelocity getAngularVelocity() {
        return m_leaderMotor.getVelocity().getValue();
    }

    // /**
    // * Gets the current flywheel velocity.
    // *
    // * @return the current linear velocity
    // */
    // public LinearVelocity getLinearVelocity() {
    // throw new UnsupportedOperationException("getLinearVelocity is not implemented
    // yet");
    // }

    // /**
    // * Creates a command to set the flywheel speed based on linear velocity.
    // *
    // * @param speed the desired linear velocity
    // * @return the command that sets flywheel speed
    // */
    // public Command setSpeed(LinearVelocity speed) {
    // throw new UnsupportedOperationException("setSpeed with LinearVelocity
    // parameter is not implemented yet");
    // }

    // /**
    // * Starts the flywheel spinning at the default RPM, the speed at which it
    // * should spin when the shooter is not actively shooting.
    // *
    // * @return a Command that starts the flywheel at the default RPM when executed
    // */
    // public Command setDefaultRPM() {
    // return setSpeed(FlywheelConstants.DEFAULT_VELOCITY);
    // }

    public AngularVelocity getSetpointVelocity() {
        return m_velocityTorque.getVelocityMeasure();
    }

    public boolean isAtTargetRPM() {
        AngularVelocity target = m_velocityTorque.getVelocityMeasure();
        AngularVelocity current = m_leaderMotor.getVelocity().getValue();

        return current.isNear(target, FlywheelConstants.RPM_TARGET_ERROR);
    }

    public boolean isAtTargetRPM(boolean isFeeding) {
        AngularVelocity target = m_velocityTorque.getVelocityMeasure();
        AngularVelocity current = getAngularVelocity();

        if (isFeeding) {
            return current.isNear(target, FlywheelConstants.RPM_TARGET_ERROR_WHILE_FEEDING);
        } else {
            return current.isNear(target, FlywheelConstants.RPM_TARGET_ERROR);
        }
    }

    // public AngularVelocity getTargetVelocity(Distance distanceToTarget) {
    // throw new UnsupportedOperationException("getTargetVelocity is not implemented
    // yet");
    // }

    /**
     * Updates flywheel telemetry.
     */
    @Override
    public void periodic() {
        // if (Constants.TELEMETRY && !DriverStation.isFMSAttached()) {
        // SmartDashboard.putNumber("FlywheelMech/linearVelocity (fps)",
        // getLinearVelocity().in(FeetPerSecond));
        // }

        SmartDashboard.putNumber("FlywheelMech/velocity (RPM)", getAngularVelocity().in(RPM));
        SmartDashboard.putNumber("FlywheelMech/setpoint (RPM)", getSetpointVelocity().in(RPM));
    }

    @Override
    public void simulationPeriodic() {
    }
}
