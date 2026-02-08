package frc.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

/**
 * Climber subsystem for the 2026 FRC game.
 *
 * Controls a two-motor arm used to climb to fixed ladder heights.
 * Current focus is Level 1, with support for Levels 2 and 3.
 */
public class ClimbSubsystem extends SubsystemBase {

    /**
     * Climb levels with predefined heights.
     */
    public enum ClimbLevel {
        LEVEL_1(ClimbConstants.kLevel1Height),
        LEVEL_2(ClimbConstants.kLevel2Height),
        LEVEL_3(ClimbConstants.kLevel3Height);

        private final double height;

        ClimbLevel(double height) {
            this.height = height;
        }

        /**
         * @return target height for this level (meters)
         */
        public double getHeight() {
            return height;
        }
    }

    // Left and right climber motors
    private final TalonFX m_leftMotor;
    private final TalonFX m_rightMotor;

    // Target climb height (meters)
    private double m_targetHeight;

    // Motor control modes
    private final PositionVoltage m_positionControl;
    private final DutyCycleOut m_dutyCycleControl;

    /**
     * Creates the climber subsystem and configures motors.
     */
    public ClimbSubsystem() {
        m_leftMotor = new TalonFX(ClimbConstants.CLIMBER_LEFT_MOTOR_CAN_ID);
        m_rightMotor = new TalonFX(ClimbConstants.CLIMBER_RIGHT_MOTOR_CAN_ID);

        m_positionControl = new PositionVoltage(0);
        m_dutyCycleControl = new DutyCycleOut(0);

        configureMotor(
                m_leftMotor,
                ClimbConstants.LEFT_MOTOR_STATOR_CURRENT_LIMIT,
                ClimbConstants.LEFT_MOTOR_SUPPLY_CURRENT_LIMIT);

        configureMotor(
                m_rightMotor,
                ClimbConstants.RIGHT_MOTOR_STATOR_CURRENT_LIMIT,
                ClimbConstants.RIGHT_MOTOR_SUPPLY_CURRENT_LIMIT);

        m_targetHeight = 0.0;
    }

    /**
     * Applies current limits and brake mode to a motor.
     *
     * @param motor       motor to configure
     * @param statorLimit stator current limit (amps)
     * @param supplyLimit supply current limit (amps)
     */
    
     private void configureMotor(TalonFX motor, double statorLimit, double supplyLimit) {
        TalonFXConfiguration config = new TalonFXConfiguration();

        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
        currentLimits.StatorCurrentLimit = statorLimit;
        currentLimits.StatorCurrentLimitEnable = true;
        currentLimits.SupplyCurrentLimit = supplyLimit;
        currentLimits.SupplyCurrentLimitEnable = true;

        config.CurrentLimits = currentLimits;

        motor.getConfigurator().apply(config);
        motor.setNeutralMode(NeutralModeValue.Brake);
        motor.setPosition(0);
    }

    /**
     * Moves the climber to a selected level.
     *
     * @param level desired climb level
     */
    public void climbToLevel(ClimbLevel level) {
        m_targetHeight = level.getHeight();

        double targetRotations = heightToRotations(m_targetHeight);

        m_leftMotor.setControl(m_positionControl.withPosition(targetRotations));
        m_rightMotor.setControl(m_positionControl.withPosition(targetRotations));
    }


    // Stops both climber motors.
    public void stopClimb() {
        m_leftMotor.setControl(m_dutyCycleControl.withOutput(0));
        m_rightMotor.setControl(m_dutyCycleControl.withOutput(0));
    }

    /**
     * Checks if the climber is at its target height.
     *
     * @return true if within tolerance
     */
    public boolean atTargetLevel() {
        double currentHeight = rotationsToHeight(m_leftMotor.getPosition().getValueAsDouble());

        return Math.abs(currentHeight - m_targetHeight) < ClimbConstants.kPositionTolerance;
    }

    /**
     * @return current climber height (meters)
     */
    public double getCurrentHeight() {
        return rotationsToHeight(
                m_leftMotor.getPosition().getValueAsDouble());
    }

    /**
     * @return target climber height (meters)
     */
    public double getTargetHeight() {
        return m_targetHeight;
    }

    /**
     * Converts height to motor rotations.
     *
     * @param heightMeters height in meters
     * @return motor rotations
     */
    private double heightToRotations(double heightMeters) {
        return heightMeters * ClimbConstants.kHeightToRotationsConversion;
    }

    /**
     * Converts motor rotations to height.
     *
     * @param rotations motor rotations
     * @return height in meters
     */
    private double rotationsToHeight(double rotations) {
        return rotations / ClimbConstants.kHeightToRotationsConversion;
    }

    /**
     * Runs every 20 ms for telemetry.
     */
    @Override
    public void periodic() {
        SmartDashboard.putNumber(
                "Climb/Current Height",
                getCurrentHeight());
        SmartDashboard.putNumber(
                "Climb/Target Height",
                getTargetHeight());
        SmartDashboard.putBoolean(
                "Climb/At Target",
                atTargetLevel());
    }
}
