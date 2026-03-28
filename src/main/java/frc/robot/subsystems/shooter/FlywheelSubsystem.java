// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.FlywheelConstants;
import frc.robot.Constants.ShooterConstants.ShooterZone;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class FlywheelSubsystem extends SubsystemBase {

    private final TalonFX m_leaderMotor;
    private final TalonFX m_followerMotor;

    private final SmartMotorController m_smartMotorController;

    private final FlyWheel m_flywheel;

    private final Debouncer m_atRPMDebouncer = new Debouncer(
            FlywheelConstants.AT_RPM_DEBOUNCE_TIME.in(Seconds),
            Debouncer.DebounceType.kRising);

    public FlywheelSubsystem() {
        m_leaderMotor = new TalonFX(FlywheelConstants.LEADER_MOTOR_CAN_ID);
        m_followerMotor = new TalonFX(FlywheelConstants.FOLLOWER_MOTOR_CAN_ID);

        SmartMotorControllerConfig smcConfig = FlywheelConstants.SMC_CONFIG
                .apply(new SmartMotorControllerConfig(this))
                .withFollowers(Pair.of(m_followerMotor, FlywheelConstants.FOLLOWER_MOTOR_INVERTED));

        m_smartMotorController = new TalonFXWrapper(
                m_leaderMotor,
                FlywheelConstants.MOTOR,
                smcConfig);

        FlyWheelConfig flywheelConfig = FlywheelConstants.FLYWHEEL_CONFIG
                .apply(new FlyWheelConfig(m_smartMotorController));

        m_flywheel = new FlyWheel(flywheelConfig);
    }

    /**
     * Creates a SysId characterization command for the flywheel.
     *
     * @return the SysId command
     */
    public Command sysId() {
        return m_flywheel.sysId(
                        Volts.of(12), Volts.of(1).per(Second), Second.of(10))
                .beforeStarting(
                        () -> SignalLogger.start())
                .finallyDo(() -> SignalLogger.stop());
    }

    /**
     * Gets the current flywheel velocity.
     *
     * @return the current angular velocity
     */
    public AngularVelocity getAngularVelocity() {
        return m_flywheel.getSpeed();
    }

    /**
     * Gets the current flywheel velocity.
     *
     * @return the current linear velocity
     */
    public LinearVelocity getLinearVelocity() {
        return m_flywheel.getLinearVelocity();
    }

    /**
     * Creates a command to set the flywheel velocity.
     *
     * @param speed the target angular velocity
     * @return the command that sets flywheel speed
     */
    public Command setSpeed(AngularVelocity speed) {
        return m_flywheel.setSpeed(speed);
    }

    /**
     * Creates a command to set the flywheel velocity from a supplier.
     *
     * @param speed the supplier of target angular velocity
     * @return the command that sets flywheel speed
     */
    public Command setSpeed(Supplier<AngularVelocity> speed) {
        return m_flywheel.setSpeed(speed);
    }

    /**
     * Creates a command to set the flywheel speed based on linear velocity.
     *
     * @param speed the desired linear velocity
     * @return the command that sets flywheel speed
     */
    public Command setSpeed(LinearVelocity speed) {
        return m_flywheel.setSpeed(RotationsPerSecond
                .of(speed.in(MetersPerSecond) / FlywheelConstants.DIAMETER_INCHES.times(Math.PI).in(Meters)));
    }

    /**
     * Starts the flywheel spinning at the default RPM, the speed at which it
     * should spin when the shooter is not actively shooting.
     *
     * @return a Command that starts the flywheel at the default RPM when executed
     */
    public Command setDefaultRPM() {
        return setSpeed(FlywheelConstants.DEFAULT_VELOCITY);
    }

    public Optional<AngularVelocity> getSetpointVelocity() {
        Optional<AngularVelocity> setpoint = m_smartMotorController.getMechanismSetpointVelocity();

        if (!setpoint.isPresent())
            return Optional.empty();

        // Convert from output to input velocity for comparison with actual velocity
        return Optional.of(setpoint.get().times(FlywheelConstants.GEARBOX.getInputToOutputConversionFactor()));
    }

    public boolean isAtTargetRPM() {
        Optional<AngularVelocity> setpoint = getSetpointVelocity();

        if (!setpoint.isPresent())
            return false;

        return m_atRPMDebouncer.calculate(
                setpoint.get().times(FlywheelConstants.GEARBOX.getOutputToInputConversionFactor()).isNear(
                        getAngularVelocity(),
                        FlywheelConstants.RPM_TARGET_ERROR));
    }

    public boolean isAtTargetRPM(boolean isFeeding) {
        Optional<AngularVelocity> setpoint = getSetpointVelocity();

        if (!setpoint.isPresent())
            return false;

        if (isFeeding) {
            return m_atRPMDebouncer.calculate(
                    setpoint.get().times(FlywheelConstants.GEARBOX.getOutputToInputConversionFactor()).isNear(
                            getAngularVelocity(),
                            FlywheelConstants.RPM_TARGET_ERROR_WHILE_FEEDING));
        }

        return m_atRPMDebouncer.calculate(
                setpoint.get().times(FlywheelConstants.GEARBOX.getOutputToInputConversionFactor()).isNear(
                        getAngularVelocity(),
                        FlywheelConstants.RPM_TARGET_ERROR));
    }

    public Command stop() {
        return this.runOnce(() -> {
            m_smartMotorController.stopClosedLoopController();
            m_smartMotorController.setDutyCycle(0);
        });
    }

    /**
     * Updates flywheel telemetry.
     */
    @Override
    public void periodic() {
        m_flywheel.updateTelemetry();

        if (Constants.TELEMETRY && !DriverStation.isFMSAttached()) {
            SmartDashboard.putNumber("FlywheelMech/linearVelocity (fps)", getLinearVelocity().in(FeetPerSecond));
        }

        SmartDashboard.putNumber("FlywheelMech/velocity (RPM)", getAngularVelocity().in(RPM));
        SmartDashboard.putNumber("FlywheelMech/setpoint (RPM)",
                getSetpointVelocity().map(
                                setpoint -> setpoint.in(RPM) * FlywheelConstants.GEARBOX.getOutputToInputConversionFactor())
                        .orElse(Double.NaN));
    }

    @Override
    public void simulationPeriodic() {
        // Run the flywheel simulation step
        m_flywheel.simIterate();
    }

    public AngularVelocity getTargetVelocity(Distance distanceToTarget) {
        ShooterZone zone = ShooterConstants.MIN_DISTANCE_TO_FLYWHEEL_SPEED_ZONE.entrySet().stream()
                .filter(entry -> distanceToTarget.in(Meters) >= entry.getKey().in(Meters))
                .max((a, b) -> Double.compare(a.getKey().in(Meters), b.getKey().in(Meters)))
                .map(Map.Entry::getValue)
                .orElse(ShooterZone.ZONE_1);
        return ShooterConstants.SHOOTER_MIN_DISTANCE_TO_FLYWHEEL_RPM.getOrDefault(zone,
                FlywheelConstants.DEFAULT_VELOCITY);
    }
}
