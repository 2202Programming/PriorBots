package frc.robot2019.commands.arm;

import edu.wpi.first.wpilibj2.command.TimedCommand;
import frc.robot2019.Robot;
import frc.robot2019.subsystems.ArmSubsystem;

public class EmptyArmCommand extends TimedCommand {
    ArmSubsystem arm = Robot.arm;

    public EmptyArmCommand(double time) {
        super(time);
        addRequirements(arm);
    }

}
