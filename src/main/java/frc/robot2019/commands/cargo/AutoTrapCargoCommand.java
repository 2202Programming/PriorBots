package frc.robot2019.commands.cargo;

import edu.wpi.first.wpilibj2.command.CommandGroup;

public class AutoTrapCargoCommand extends CommandGroup {
    public AutoTrapCargoCommand() {
        addSequential(new DeployCargoTrapCommand());
    }
}