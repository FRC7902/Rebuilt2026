package frc.robot.subsystems;


import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class Indexer extends SubsystemBase{
    private final TalonFX m_indexerMotor;
    private double currentSpeed = 0.0;
    public Indexer() {
        m_indexerMotor = new TalonFX(IndexerConstants.INDEXER_MOTOR_CAN_ID);

        TalonFXConfigurator indexerConfig = m_indexerMotor.getConfigurator();
        CurrentLimitsConfigs limitsConfigs = new CurrentLimitsConfigs();

        limitsConfigs.StatorCurrentLimit = 50.0;
        limitsConfigs.StatorCurrentLimitEnable = true;

        limitsConfigs.SupplyCurrentLimit = 30.0;
        limitsConfigs.SupplyCurrentLimitEnable = true;

        indexerConfig.apply(limitsConfigs);
    }

    public void setSpeed(double speed) {
        currentSpeed = speed;
        m_indexerMotor.set(speed);
    }

    public double getSpeed() {
        return currentSpeed;
    }
    public Command suckBalls(double speed){
        return runOnce(() -> setSpeed(speed));
    }
    public Command spitBalls(double speed) {
        return runOnce(() -> setSpeed(-speed));
    }
    public Command stopBalls() {
        return runOnce(() -> setSpeed(0));
    }
}