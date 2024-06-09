package frc.robot2023.commands.EndEffector;


import edu.wpi.first.wpilibj2.command.Command;
import frc.base.RobotContainerOrig;
import frc.robot2023.subsystems.Claw_Substyem;

public class WheelsIn extends Command {
    Claw_Substyem claw = RobotContainerOrig.RC().claw;
    
    public WheelsIn() {
        // don't care do nothing
    }

    @Override
    public void initialize() {
        claw.wheelsIn();
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {return false;}

    @Override
    public void end(boolean interrupted) {
        claw.wheelsOff();
    }
}
