package frc.robot2019.commands.intake.tests;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib2202.builder.RobotContainer;
import frc.robot2019.subsystems.IntakeSubsystem;

/**
 * VacuumCommand (enable) true - turn it on false - turn it off
 * 
 * Instantiate with one or other for what you need.
 *  Use intake.kRelease or kVacuum 
 * 
 */
public class SolenoidTestCommand extends InstantCommand {
    // Current state
    boolean vacuumCmd;
    final IntakeSubsystem intake;

    public SolenoidTestCommand(boolean vacuumCmd) {
        intake = RobotContainer.getSubsystem(IntakeSubsystem.class);
        addRequirements(intake.getVacuumSubsystem());
        this.setName("solenoidTest=" + vacuumCmd);
        this.vacuumCmd = vacuumCmd;
    }

    @Override
    public void execute() {
        intake.releaseSolenoid(vacuumCmd);
    }
}
