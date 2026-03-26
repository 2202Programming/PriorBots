// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.timbot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.timbot.subsystem.ShooterLifter;
import frc.timbot.subsystem.Feeder;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FeedFrisbee extends Command {
  /** Creates a new FeedFrisbee. */
  
  final Feeder feeder;

  public FeedFrisbee() {
    // Use addRequirements() here to declare subsystem dependencies.

  
    feeder = RobotContainer.getSubsystem(Feeder.class);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  // Called once the command ends or is interrupted.

  feeder.feeder_fire();
  }
  
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
