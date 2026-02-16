package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.IndexerConstants;

public class Indexer {
    private final TalonFX m_indexerMotor;
    private double currentSpeed = 0.0;

    public Indexer() {
        m_indexerMotor = new TalonFX(IndexerConstants.INDEXER_MOTOR_CAN_ID);
    }

    public void setSpeed(double speed) {
        currentSpeed = speed;
        m_indexerMotor.set(speed);
    }

    public double getSpeed() {
        return currentSpeed;
    }
}