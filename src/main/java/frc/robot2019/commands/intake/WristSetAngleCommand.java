package frc.robot2019.commands.intake;

import edu.wpi.first.wpilibj2.command.TimedCommand;
import frc.robot2019.Robot;

public class WristSetAngleCommand extends TimedCommand {
    private double angle;

    /**
     * Sets the wrist to a specific angle
     */
    public WristSetAngleCommand(double angle) {
        this(angle, 0.0);
    }

    public WristSetAngleCommand(double angle, double timeout) {
        super(timeout);
        addRequirements(Robot.intake);
        this.angle = angle;
    }

    @Override
    public void initialize() {
        execute();
    }

    @Override
    public void execute() {
        // intake angle is relative to servo
        Robot.intake.setAngle(angle);
    }
}