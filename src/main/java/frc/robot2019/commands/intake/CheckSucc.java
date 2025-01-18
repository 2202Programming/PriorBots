package frc.robot2019.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2019.subsystems.IntakeSubsystem;

/**
 * Stops the suctions cup's solenoid from being stuck in indeterminant state
 */
public class CheckSucc extends Command {
    final IntakeSubsystem intake;
    public CheckSucc() {
        intake = RobotContainer.getSubsystem(IntakeSubsystem.class);
        addRequirements(intake);
        this.withTimeout(0.2);   //TODO - dpl 1/17/25 confirm this approach works
    }

    @Override
    public void initialize() {
        // setTimeout(0.2);  replace with withTimeout decorator
        intake.releaseSolenoid(true);
    }
    /*
    @Override
    public boolean isFinished() {
        return isTimedOut();
    }
    */

    public void end(boolean interrupted) {
        intake.releaseSolenoid(false);
    }
}