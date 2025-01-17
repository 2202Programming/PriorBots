package frc.robot2019.commands.arm.tests;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot2019.Robot;

public class TestRotateArmToAngleCommand extends Command {
    final private double kTolerance = 2.0;
    private double angle;
    private double timeout;

    public TestRotateArmToAngleCommand(double angle, double timeout) {
        this.angle = angle;
        this.timeout = timeout;
        addRequirements(Robot.arm);
    }
    
    public TestRotateArmToAngleCommand(double angle)
    {
        this(angle, 10);
    }

    @Override
    public void initialize() {
        setTimeout(timeout);
    }
    @Override
    public void execute() {
        Robot.arm.setAngle(angle);
    }

    public boolean isFinished() {
        boolean pos = Math.abs(Robot.arm.getRealAngle() - angle) < kTolerance; 
        return pos || isTimedOut();  
    }
}