package frc.robot2019.commands.climb.tests;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot2019.Robot;

/**
 * 
 * Instantiate with one or other for what you need.
 */
public class PawlSolenoidTestCmd extends Command {
    // On state
    boolean enabled;

    public PawlSolenoidTestCmd(boolean enabled) {
        this.setName("PawlTest=" + enabled);
        this.enabled = enabled;
    }

    @Override
    public void initialize() {
        Robot.climber.setPawl(enabled);
    }

    @Override
    public void execute() {      
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
