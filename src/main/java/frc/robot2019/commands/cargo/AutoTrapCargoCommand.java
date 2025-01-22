package frc.robot2019.commands.cargo;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoTrapCargoCommand extends SequentialCommandGroup {
    public AutoTrapCargoCommand() {
        addCommands(new DeployCargoTrapCommand());
    }
}