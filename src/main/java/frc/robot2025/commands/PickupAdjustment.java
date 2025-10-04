// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2025.subsystems.EndEffector_Subsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PickupAdjustment extends Command {
  EndEffector_Subsystem ee_Subsystem;
  final double DELAY_COUNT = 5;
  double count;
  /** Creates a new PickupAdjustment. */
  public PickupAdjustment() {
    ee_Subsystem = RobotContainer.getSubsystem(EndEffector_Subsystem.class);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ee_Subsystem.setPercent(-0.6);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(ee_Subsystem.pieceReady()){
    count++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return count >= DELAY_COUNT;
  }
}
