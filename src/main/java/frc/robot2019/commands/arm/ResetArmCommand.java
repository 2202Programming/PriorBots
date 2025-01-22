package frc.robot2019.commands.arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib2202.builder.RobotContainer;
import frc.robot2019.subsystems.ArmSubsystem;
public class ResetArmCommand extends InstantCommand {
  final ArmSubsystem arm;
  public ResetArmCommand() {
    arm = RobotContainer.getSubsystem(ArmSubsystem.class);
    addRequirements(arm);

  }

  @Override
  public void execute() {
    arm.resetArm(0.0);
  }
}
