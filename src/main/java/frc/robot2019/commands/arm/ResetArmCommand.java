package frc.robot2019.commands.arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot2019.Robot;

public class ResetArmCommand extends InstantCommand {
  public ResetArmCommand() {
    addRequirements(Robot.arm);
  }

  @Override
  public void execute() {
    Robot.arm.resetArm(0.0);
  }
}
