/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot2019.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot2019.Robot;
import frc.robot2019.subsystems.ArmSubsystem;

public class ArmZero extends Command {
  ArmSubsystem arm = Robot.arm;

  /**
   * Creates an ArmZero command. This command zeros the arm's encoder.
   */
  public ArmZero() {
    addRequirements(arm);
  }

  /**
   * Zeros the arm's encoders - arm and extension are at starting point
   */
  @Override
  public void initialize() {
    arm.zeroArm();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
