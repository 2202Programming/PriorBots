package frc.robot2019.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2019.subsystems.IntakeSubsystem;

/**
 * VacuumCommand (enable)
 * true - turn it on - effectively an instanct
 * false - turn it off - will run duration of timeout
 * - leaves the pump off, but will put solenoid to vac mode
 * so compressed air won't drain.
 * 
 * Instantiate with one or other for what you need.
 */
public class VacuumCommand extends Command {
    boolean enable;
    boolean done = false;
    double timeout;
    final IntakeSubsystem intake;
    final Timer timer;

    public VacuumCommand(boolean enable, double timeout) {
        intake = RobotContainer.getSubsystem(IntakeSubsystem.class);
        // addRequirements(intake.getVacuumSubsystem());
        this.setName("vac=" + enable);
        this.enable = enable;
        this.timeout = timeout;
        timer = new Timer();
    }

    @Override
    public void initialize() {
        done = false; // turning off takes time.
        timer.start();

        if (enable) {
            done = true; // this is instant
            intake.setVacuum(true);
        } else {
            // pump off, flip the solenoid to put compressed air into lines
            intake.setVacuum(false);
            intake.releaseSolenoid(true); // powered, will release payload
        }
    }

    // When we release we blow air when we retract, sensor goes below baseline -
    // problem...
    @Override
    public boolean isFinished() {
        // put the solenoid back to vacuum on timeout.
        if (timer.hasElapsed(timeout)) {
            intake.releaseSolenoid(false); // unpowerered for vacuum
        }
        return done || timer.hasElapsed(timeout);   // || isTimedOut();
    }

}
