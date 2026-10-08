package frc.robot2026.command;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.lib2202.builder.RobotContainer;
import frc.robot2026.subsystems.Hopper;
import frc.robot2026.subsystems.Intake;

public class AgitateOS extends Command {
    final double HopperSpeedDefault = 0.25;
    final Intake intake;
    final Hopper hopper;
    final Timer periodTimer;
    final Timer delayTimer;
    final double period;
    final double delay;
    final boolean hopperForward;
    final double intake_spd;      

    double in_spd, hop_spd;
    /**
     * AgitateOS - old school implementation, a work around for
     * use in a parallel cmd group in auto. See ncShoot()
     * 
     * @param hopperForward only run hopper forward
     * @param period        time to wait for switching dir
     * @param intake_spd    pct to drive intake 
     */
    public AgitateOS(boolean hopperForward, double hop_spd, double period, double delay, double intake_spd) {
        this.intake = RobotContainer.getSubsystem("intake");
        this.hopper = RobotContainer.getSubsystem(Hopper.class);
        this.period = period;
        this.delay = delay;
        this.hop_spd = hop_spd;
        this.hopperForward = hopperForward;
        this.intake_spd = Math.abs(intake_spd);  // always start going in.
        this.periodTimer = new Timer();
        this.delayTimer = new Timer();
        addRequirements(intake, hopper);
    }
    
    // a few shortcut options
    public AgitateOS(double period, double hop_spd, double delay, double intake_spd) {
        this(false, hop_spd, period, delay, intake_spd);
    }
    
    public AgitateOS(double period) {
        this(false, 0.25, period, 0.35, 0.65);
    }

    /**
     * The initial subroutine of a command. Called once when the command is
     * initially scheduled.
     */
    @Override
    public void initialize() {
        in_spd = intake_spd;
        //set the hardware to start speeds
        intake.setPercent(in_spd);
        hopper.setBeltPct(hop_spd);
        periodTimer.restart();
        delayTimer.reset();
        //System.out.println("AgitateOS start ");    
    }
    @Override
    public void execute() {
        if(delayTimer.isRunning() && !delayTimer.hasElapsed(delay)) {
            return;
        } else if (delayTimer.hasElapsed(delay) && delayTimer.isRunning()) {
            //set the hardware to new speeds
            intake.setPercent(in_spd);
            hopper.setBeltPct(hop_spd);
            delayTimer.stop();
            periodTimer.restart();
        }

        // check timer for direction flip time
        if (periodTimer.hasElapsed(period)) {
            in_spd = in_spd * -1.0;
            hop_spd = (hopperForward) ? HopperSpeedDefault : hop_spd * -1.0;

            intake.setPercent(0.0);
            hopper.setBeltPct(0.0);
            
            delayTimer.restart();
        }
        // this is the part that was breaking the original implementation
        // needed to take over watching light gate, see getInterruptionBehavior().
        if (intake.hasFuel()) {
            //reset intake to going forward
            in_spd = intake_spd;
            intake.setPercent(in_spd);
        }
    }

    // stop everything when canceled
    @Override
    public void end(boolean interrupted) {
        intake.setPercent(0.0);
        hopper.setBeltPct(0.0);    
        //System.out.println("AgitateOS end "+ interrupted);    
    }

    // this command NEVER finishes, it will be unscheduled, interrupted when done.
    @Override
    public boolean isFinished() {
        return false;
    }

    // we have to cancel incoming otherwise the Intake HasFuel trigger will
    // go off and cause us to end. This prevents it so this command can handle 
    // the intake's behavior completely.
    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }

}