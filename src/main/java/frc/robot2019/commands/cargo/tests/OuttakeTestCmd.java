package frc.robot2019.commands.cargo.tests;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2019.subsystems.CargoTrapSubsystem;

/**
 * VacuumCommand (enable) true - turn it on false - turn it off
 * 
 * Instantiate with one or other for what you need.
 */
public class OuttakeTestCmd extends Command {
    // On state
    final double speed;
    final CargoTrapSubsystem cargoTrap;

    public OuttakeTestCmd(double speed) {
        cargoTrap = RobotContainer.getSubsystem(CargoTrapSubsystem.class);
        this.setName("Cargo Motor=" + -Math.abs(speed));
        this.speed = -Math.abs(speed);
    }

    @Override
    public void initialize() {
        cargoTrap.setIntake(speed);
    }

    @Override
    public void execute() {
        cargoTrap.setIntake(speed);
    }

    @Override
    public void end(boolean interrupted) {
        cargoTrap.setIntake(0);        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
