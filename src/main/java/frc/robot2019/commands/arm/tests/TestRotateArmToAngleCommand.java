package frc.robot2019.commands.arm.tests;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib2202.builder.RobotContainer;
import frc.robot2019.subsystems.ArmSubsystem;

public class TestRotateArmToAngleCommand extends WaitCommand {
    final private double kTolerance = 2.0;
    private double angle;
    final ArmSubsystem arm;

    public TestRotateArmToAngleCommand(double angle, double timeout) {
        super(timeout);
        arm = RobotContainer.getSubsystem(ArmSubsystem.class);
        this.angle = angle;
        addRequirements(arm);
    }
    
    public TestRotateArmToAngleCommand(double angle)
    {
        this(angle, 10);
    }

    @Override
    public void initialize() {
        super.initialize(); // setTimeout(timeout);
    }
    @Override
    public void execute() {
        arm.setAngle(angle);
    }

    public boolean isFinished() {
        boolean pos = Math.abs(arm.getRealAngle() - angle) < kTolerance; 
        return pos || super.isFinished();  
    }
}