package frc.robot2019.commands.cargo;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib2202.builder.RobotContainer;
import frc.robot2019.subsystems.CargoTrapSubsystem;


public class DeployCargoTrapCommand extends InstantCommand {
    final CargoTrapSubsystem cargoTrap;

    public DeployCargoTrapCommand() {
        cargoTrap = RobotContainer.getSubsystem(CargoTrapSubsystem.class);
        addRequirements(cargoTrap);
    }

    @Override
    public void execute() {
        cargoTrap.deployTrap();
    }
}