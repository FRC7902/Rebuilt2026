package commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.subsystems.ClimbSubsystem;
import frc.subsystems.ClimbSubsystem.ClimbLevel;
import frc.robot.Constants.ClimbConstants;

// Command to climb to a specific level (L1, L2, or L3)
// Includes 30-second timeout for safety (CLMB-05)
public class ClimbToLevelCommand extends Command {
    
    private final ClimbSubsystem m_climbSubsystem;
    private final ClimbLevel m_targetLevel;
    private final Timer m_timer;
    
    public ClimbToLevelCommand(ClimbSubsystem climbSubsystem, ClimbLevel targetLevel) {
        m_climbSubsystem = climbSubsystem;
        m_targetLevel = targetLevel;
        m_timer = new Timer();
        
        addRequirements(climbSubsystem);
    }
    
    // Factory method for Level 1
    public static ClimbToLevelCommand climbToL1(ClimbSubsystem climbSubsystem) {
        return new ClimbToLevelCommand(climbSubsystem, ClimbLevel.LEVEL_1);
    }
    
    // Factory method for Level 2 (TENTATIVE)
    public static ClimbToLevelCommand climbToL2(ClimbSubsystem climbSubsystem) {
        return new ClimbToLevelCommand(climbSubsystem, ClimbLevel.LEVEL_2);
    }
    
    // Factory method for Level 3 (TENTATIVE)
    public static ClimbToLevelCommand climbToL3(ClimbSubsystem climbSubsystem) {
        return new ClimbToLevelCommand(climbSubsystem, ClimbLevel.LEVEL_3);
    }
    
    @Override
    public void initialize() {
        m_timer.restart();
        m_climbSubsystem.climbToLevel(m_targetLevel);
        System.out.println("ClimbToLevelCommand: Starting climb to " + m_targetLevel);
    }
    
    @Override
    public void execute() {
        // Climbing handled by subsystem position control
    }
    
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            m_climbSubsystem.stopClimb();
            System.out.println("ClimbToLevelCommand: Interrupted!");
        } else {
            System.out.println("ClimbToLevelCommand: Reached " + m_targetLevel + 
                             " in " + m_timer.get() + " seconds");
        }
        
        m_timer.stop();
    }
    
    @Override
    public boolean isFinished() {
        boolean atTarget = m_climbSubsystem.atTargetLevel();
        boolean timedOut = m_timer.hasElapsed(ClimbConstants.kMaxClimbTime);
        
        if (timedOut && !atTarget) {
            System.err.println("ClimbToLevelCommand: TIMEOUT! Failed to reach " + 
                             m_targetLevel + " within " + ClimbConstants.kMaxClimbTime + " seconds");
        }
        
        return atTarget || timedOut;
    }
}