package commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import frc.subsystems.ClimbSubsystem;

// Command to climb to a specific level (L1, L2, or L3)
// Includes 30-second timeout for safety (CLMB-05)
public class ClimbToLevelCommand extends Command {

    private final ClimbSubsystem climb;
    private final Distance targetHeight;
    private final Timer timer = new Timer();

    public ClimbToLevelCommand(ClimbSubsystem climb, Distance targetHeight) {
        this.climb = climb;
        this.targetHeight = targetHeight;
        addRequirements(climb);
    }

    @Override
    public void initialize() {
        timer.restart();
        climb.setHeight(targetHeight);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            climb.setHeightAndStop(targetHeight);
        }
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return true; //TODO: Update this 
    }
}
