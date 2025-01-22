package frc.robot2019.commands.intake.tests;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib2202.builder.RobotContainer;
import frc.robot2019.subsystems.IntakeSubsystem;


/**
 * VacuumCommand (enable) true - turn it on false - turn it off
 * 
 * Instantiate with one or other for what you need.
 */
public class VacuumTestCommand extends InstantCommand {
    // Current state
    boolean enabled;
    final IntakeSubsystem intake;
   
    public VacuumTestCommand(boolean enabled) {
        intake = RobotContainer.getSubsystem(IntakeSubsystem.class);
        addRequirements(intake.getVacuumSubsystem());
        this.setName("vac=" + enabled);
        this.enabled = enabled;
    }

    @Override
    public void execute() {
        if (enabled) {
            // Transition to Off
            intake.setVacuum(true);
        } else {
            // Transition to On
            intake.setVacuum(false);
        }
        enabled = !enabled;
    }
}
