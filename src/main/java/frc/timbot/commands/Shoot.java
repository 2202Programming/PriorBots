package frc.timbot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.timbot.subsystem.FlywheelSubsystem;
import frc.timbot.utils.Stick;

public class Shoot extends Command {

    private FlywheelSubsystem flywheel;

    public Shoot(FlywheelSubsystem flywheel) {
        this.flywheel = flywheel;
    }

    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override 
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if(flywheel.isAtSpeed(0.01)) {
            return true;
        }
        return false;
    }
}
