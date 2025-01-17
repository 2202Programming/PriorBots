package frc.robot2019.commands.intake.tests;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot2019.Robot;

/**
 * VacuumCommand (enable) true - turn it on false - turn it off
 * 
 * Instantiate with one or other for what you need.
 */
public class IntakeTestCommand extends InstantCommand {
    // Current state
    boolean vacuumOn;

    public IntakeTestCommand(boolean vacuumOn) {
        addRequirements(Robot.intake.getVacuumSubsystem());
        this.setName("vac=" + vacuumOn);
        this.vacuumOn = vacuumOn;
    }

    @Override
    public void execute() {
        Robot.intake.setVacuum(vacuumOn);
    }
}
