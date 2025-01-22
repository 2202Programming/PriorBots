package frc.robot2019.commands.arm;

// was import edu.wpi.first.wpilibj2.command.TimedCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib2202.builder.RobotContainer;
import frc.robot2019.subsystems.ArmSubsystem;

public class EmptyArmCommand extends WaitCommand {
    final ArmSubsystem arm;
    public EmptyArmCommand(double time) {
        super(time);
        arm = RobotContainer.getSubsystem(ArmSubsystem.class);
        addRequirements(arm);
    }

}
