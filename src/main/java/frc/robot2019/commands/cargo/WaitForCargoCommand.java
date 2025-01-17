package frc.robot2019.commands.cargo;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot2019.Robot;

public class WaitForCargoCommand extends Command {
    public WaitForCargoCommand() {
        addRequirements(Robot.cargoTrap);
    }

    @Override
    public boolean isFinished() {
       return Robot.cargoTrap.cargoInSight();
    }
}