package frc.robot2019.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2019.subsystems.IntakeSubsystem;

public class IntakeZero extends Command {
    final IntakeSubsystem intake;
    public IntakeZero() {
        intake = RobotContainer.getSubsystem(IntakeSubsystem.class);
        addRequirements(intake);
    }

    
    @Override
    public void initialize() {
        intake.setAngle(intake.WristStraightDegrees);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}