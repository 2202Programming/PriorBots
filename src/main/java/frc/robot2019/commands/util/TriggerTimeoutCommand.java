package frc.robot2019.commands.util;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class TriggerTimeoutCommand extends Command {
    private BooleanSupplier event; 
    private double timeout;
    final Timer timer;

    public TriggerTimeoutCommand(BooleanSupplier func, Double timeout) {
        event = func;
        this.timeout = timeout;
        timer = new Timer();
    }

    @Override
    public void initialize() {
        timer.start(); //setTimeout(timeout);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(timeout) || event.getAsBoolean();
    }
}
