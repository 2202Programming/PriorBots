package frc.robot2019.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot2019.Robot;

public class CheckSolenoids extends Command {
    public CheckSolenoids() {
        addRequirements(Robot.climber);
    }
    
    public void initialize() {
        setTimeout(0.1);
        Robot.climber.setDrawerSlide(true);
        Robot.climber.setPawl(true);
    }

    public boolean isFinished() {
        return isTimedOut();
    }

    public void end(boolean interrupted) {
        Robot.climber.setDrawerSlide(false);
        Robot.climber.setPawl(false);
    }
}