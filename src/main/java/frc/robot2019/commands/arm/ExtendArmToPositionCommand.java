package frc.robot2019.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2019.subsystems.ArmSubsystem;

public class ExtendArmToPositionCommand extends Command {
    final private double kTolerance = 0.50;  //(inches)
    private double distance;
    final ArmSubsystem arm;
    public ExtendArmToPositionCommand(double distance) {
        this.distance = distance;
        arm = RobotContainer.getSubsystem(ArmSubsystem.class);
        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.setExtension(distance);
    }

    public boolean isFinished() {
        return Math.abs(arm.getExtension() - distance) < kTolerance;   
    }
}