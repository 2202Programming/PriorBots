package frc.robot2023.commands.EndEffector;


import edu.wpi.first.wpilibj2.command.Command;
import frc.base.RobotContainerOrig;
import frc.robot2023.subsystems.Claw_Substyem;

public class ToggleClaw extends Command {
    // SSs
    Claw_Substyem claw = RobotContainerOrig.RC().claw;
    
    public ToggleClaw() {
        addRequirements(claw);
    }

    @Override
    public void initialize() {
        if (claw.isOpen()) claw.close(); else claw.open();
    }


    @Override
    public boolean isFinished() {
        return true;
    }
}