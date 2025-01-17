package frc.robot2019.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot2019.Robot;

public class RetractClimbFoot extends Command {
    public RetractClimbFoot() {
        addRequirements(Robot.climber);
    }

    public void execute() {
        Robot.climber.setExtenderSpeed(-0.3);
    }

    public void end(boolean interrupted) {
        Robot.climber.setExtenderSpeed(0);
    }

    public boolean isFinished() {
        return Robot.climber.getExtension() <= 10; //TODO: Better stop at bottom
    }
}