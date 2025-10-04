package frc.timbot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.timbot.subsystem.FlywheelSubsystem;

public class Shoot extends Command {

    FlywheelSubsystem flywheel;
    double rpm;
    Timer timer = new Timer();

    public Shoot(double rpm) {
        this.flywheel = RobotContainer.getSubsystem(FlywheelSubsystem.class);
        this.rpm = rpm;
    }

    @Override
    public void initialize() {
        flywheel.setSpeed(rpm);
        timer.stop();

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override 
    public void execute() {
        if(flywheel.isAtSpeed(0.01)) {
            timer.start();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        flywheel.setSpeed(0.0);
        //fire the solenoid
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(2.0);
    }
}
