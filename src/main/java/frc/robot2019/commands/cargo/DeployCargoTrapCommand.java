package frc.robot2019.commands.cargo;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot2019.Robot;

public class DeployCargoTrapCommand extends InstantCommand {
    public DeployCargoTrapCommand() {
        addRequirements(Robot.cargoTrap);
    }

    @Override
    public void execute() {
        Robot.cargoTrap.deployTrap();
    }
}