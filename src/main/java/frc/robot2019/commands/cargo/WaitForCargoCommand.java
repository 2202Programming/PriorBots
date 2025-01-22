package frc.robot2019.commands.cargo;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2019.subsystems.CargoTrapSubsystem;

public class WaitForCargoCommand extends Command {
    final CargoTrapSubsystem cargoTrap;
    public WaitForCargoCommand() {
        cargoTrap = RobotContainer.getSubsystem(CargoTrapSubsystem.class);
        addRequirements(cargoTrap);
    }

    @Override
    public boolean isFinished() {
       return cargoTrap.cargoInSight();
    }
}