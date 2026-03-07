package frc.robot.subsystems;


import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase{
    private final TalonFX m_indexerMotor;
    private double m_currentSpeed = 0.0;
    public IndexerSubsystem() {
        m_indexerMotor = new TalonFX(IndexerConstants.INDEXER_MOTOR_CAN_ID);

        m_indexerMotor.setNeutralMode(NeutralModeValue.Brake);

        TalonFXConfigurator indexerConfig = m_indexerMotor.getConfigurator();
        CurrentLimitsConfigs limitsConfigs = new CurrentLimitsConfigs();

        limitsConfigs.StatorCurrentLimit = 50.0;
        limitsConfigs.StatorCurrentLimitEnable = true;

        limitsConfigs.SupplyCurrentLimit = 30.0;
        limitsConfigs.SupplyCurrentLimitEnable = true;

        indexerConfig.apply(limitsConfigs);
    }

    public void setSpeed(double speed) {
        m_currentSpeed = speed;
        m_indexerMotor.set(speed);
    }

    public double getSpeed() {
        return m_currentSpeed;
    }
    public Command intaking(double speed){
        return runOnce(() -> setSpeed(speed));
    }
    public Command outtaking(double speed) {
        return runOnce(() -> setSpeed(-speed));
    }
    public Command stopBalls() {
        return runOnce(() -> setSpeed(0));
    }
    @Override
    public void periodic() {
        if (IndexerConstants.TELEMETRY){
            SmartDashboard.putNumber("Indexer - Motor Voltage", m_indexerMotor.getMotorVoltage().getValueAsDouble());
        }
    }
}