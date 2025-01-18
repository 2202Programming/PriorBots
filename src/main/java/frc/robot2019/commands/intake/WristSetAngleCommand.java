package frc.robot2019.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2019.subsystems.IntakeSubsystem;


public class WristSetAngleCommand extends Command /*TimedCommand */ {
    private double angle;
    double timeout;
    final Timer timer;
    final IntakeSubsystem intake;
    /**
     * Sets the wrist to a specific angle
     */
    public WristSetAngleCommand(double angle) {
        this(angle, 0.0);
    }

    public WristSetAngleCommand(double angle, double timeout) {
        intake = RobotContainer.getSubsystem(IntakeSubsystem.class);
        timer = new Timer();
        this.timeout = timeout;
        this.angle = angle; 
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        timer.start();
        execute();
    }

    @Override
    public void execute() {
        // intake angle is relative to servo
        intake.setAngle(angle);
    }

    @Override
    public boolean isFinished(){
        return timer.hasElapsed(timeout);
    }
}