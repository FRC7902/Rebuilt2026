package frc.robot;


import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    public Command oscillatingIntake(){
        return runOnce(() -> setSpeed(Math.sin(2*Math.PI * Timer.getFPGATimestamp()) * IndexerConstants.AGGRESIVE_MOTOR_SPEED));
    }
    @Override
    public void periodic(){
        SmartDashboard.putNumber("Speed of indexer motor", Math.sin(2*Math.PI * Timer.getFPGATimestamp()) * IndexerConstants.AGGRESIVE_MOTOR_SPEED);
        SmartDashboard.putNumber("Voltage of indexer motor",m_indexerMotor.getMotorVoltage().getValueAsDouble());
    }
}