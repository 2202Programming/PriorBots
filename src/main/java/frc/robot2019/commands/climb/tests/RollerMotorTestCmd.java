package frc.robot2019.commands.climb.tests;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot2019.Robot;

/**
 * VacuumCommand (enable) true - turn it on false - turn it off
 * 
 * Instantiate with one or other for what you need.
 */
public class RollerMotorTestCmd extends Command {
    // On state
    double speed;

    public RollerMotorTestCmd(double speed) {
        this.setName("Climb Motor=" + speed);
        this.speed = speed;
    }

    @Override
    public void initialize() {
        Robot.climber.setRollerSpeed(speed);
    }

    @Override
    public void execute() {
        Robot.climber.setRollerSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        Robot.climber.setRollerSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
