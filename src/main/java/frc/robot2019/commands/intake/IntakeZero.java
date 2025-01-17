package frc.robot2019.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot2019.Robot;


public class IntakeZero extends Command {

    public IntakeZero() {
        addRequirements(Robot.intake);
    }

    
    @Override
    public void initialize() {
        Robot.intake.setAngle(Robot.intake.WristStraightDegrees);
    }

    @Override
    public void execute() {  }

    // This is just a test, it doesn't finish. Enjoy moving the write with the
    // controller.
    @Override
    public boolean isFinished() {
        return true;
    }

}