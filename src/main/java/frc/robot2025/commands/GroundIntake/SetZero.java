// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.commands.GroundIntake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2025.subsystems.GroundIntake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetZero extends Command {
  final GroundIntake groundIntake;
  
  public SetZero() {
    // Use addRequirements() here to declare subsystem dependencies.
    this.groundIntake = RobotContainer.getSubsystem(GroundIntake.class);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    groundIntake.setZero();
  }
  
  @Override
  public boolean isFinished() {
    return true;
  }
}
