package frc.robot2019.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot2019.Robot;

public class ExtendArmToPositionCommand extends Command {
    final private double kTolerance = 0.50;  //(inches)
    private double distance;

    public ExtendArmToPositionCommand(double distance) {
        this.distance = distance;
        addRequirements(Robot.arm);
    }

    @Override
    public void execute() {
        Robot.arm.setExtension(distance);
    }

    public boolean isFinished() {
        return Math.abs(Robot.arm.getExtension() - distance) < kTolerance;   
    }
}