package frc.robot2019.commands.cargo;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2019.subsystems.CargoTrapSubsystem;

public class AutoCargoIntakeCommand extends Command {
    final private CargoTrapSubsystem cargoTrap;
    private double speed;

    public AutoCargoIntakeCommand(double speed) {
        cargoTrap = RobotContainer.getSubsystem(CargoTrapSubsystem.class);
        addRequirements(cargoTrap);
        this.speed = speed;
    }

    @Override
    public void initialize() {
        cargoTrap.deployTrap();        
    }

    @Override
    public void execute() {
        cargoTrap.setIntake(speed);
    }

    @Override
    public void end(boolean interrupted) {
        cargoTrap.retractTrap();
        cargoTrap.setIntake(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}