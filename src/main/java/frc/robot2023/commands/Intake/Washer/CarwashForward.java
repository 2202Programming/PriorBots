package frc.robot2023.commands.Intake.Washer;


import edu.wpi.first.wpilibj2.command.Command;
import frc.base.RobotContainerOrig;
import frc.robot2023.subsystems.Intake;

public class CarwashForward extends Command {

    Intake intake;

    public CarwashForward() {
        intake = RobotContainerOrig.RC().intake;
    }
    
    @Override
    public void initialize() {
        intake.carwashOn();
    }

    @Override
    public void execute() {
        // do nothing
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.carwashOff();
    }
    
}
