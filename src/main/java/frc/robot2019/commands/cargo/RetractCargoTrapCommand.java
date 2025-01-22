package frc.robot2019.commands.cargo;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib2202.builder.RobotContainer;
import frc.robot2019.subsystems.CargoTrapSubsystem;
/**
 * Retracts the cargo trap on the robot chassis.
 */
public class RetractCargoTrapCommand extends InstantCommand {
    /**
     * Constructor for the retract cargo trap command. Requires a cargoTrap subsystem.
     */
    final CargoTrapSubsystem cargoTrap;
    public RetractCargoTrapCommand() {
        cargoTrap = RobotContainer.getSubsystem(CargoTrapSubsystem.class);
        addRequirements(cargoTrap);
    }
    /**
     * Retracts the cargo trap.
     */
    @Override
    public void execute() {
        cargoTrap.retractTrap();
        cargoTrap.setIntake(0.0);
    }
}