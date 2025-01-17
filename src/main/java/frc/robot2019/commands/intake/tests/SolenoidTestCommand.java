package frc.robot2019.commands.intake.tests;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot2019.Robot;

/**
 * VacuumCommand (enable) true - turn it on false - turn it off
 * 
 * Instantiate with one or other for what you need.
 *  Use Robot.intake.kRelease or kVacuum 
 * 
 */
public class SolenoidTestCommand extends InstantCommand {
    // Current state
    boolean vacuumCmd;

    public SolenoidTestCommand(boolean vacuumCmd) {
        addRequirements(Robot.intake.getVacuumSubsystem());
        this.setName("solenoidTest=" + vacuumCmd);
        this.vacuumCmd = vacuumCmd;
    }

    @Override
    public void execute() {
        Robot.intake.releaseSolenoid(vacuumCmd);
    }
}
