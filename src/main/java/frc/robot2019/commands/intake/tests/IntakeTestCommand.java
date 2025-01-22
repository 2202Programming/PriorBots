package frc.robot2019.commands.intake.tests;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib2202.builder.RobotContainer;
import frc.robot2019.subsystems.IntakeSubsystem;

/**
 * VacuumCommand (enable) true - turn it on false - turn it off
 * 
 * Instantiate with one or other for what you need.
 */
public class IntakeTestCommand extends InstantCommand {
    // Current state
    boolean vacuumOn;
    final IntakeSubsystem intake;

    public IntakeTestCommand(boolean vacuumOn) {
        intake = RobotContainer.getSubsystem(IntakeSubsystem.class);
        addRequirements(intake.getVacuumSubsystem());
        this.setName("vac=" + vacuumOn);
        this.vacuumOn = vacuumOn;
    }

    @Override
    public void execute() {
        intake.setVacuum(vacuumOn);
    }
}
