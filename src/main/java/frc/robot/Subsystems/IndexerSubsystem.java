package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.IndexerConstants;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

public class IndexerSubsystem {
    private final TalonFX m_motor;
    

    public IndexerSubsystem() {
        m_motor = new TalonFX(IndexerConstants.INDEXER_MOTOR_CAN_ID);
    }

    public void setSpeed(double speed) {
        m_motor.set(speed);
    }
}