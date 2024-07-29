// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2024.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2024.subsystems.Climber;

public class ClimberMoveCalibrate extends Command {
  Climber climber;
  final int DELAY_COUNT = 3; // frames to move before watching for stall
  int count;
  final double tolerance = 0.25;

  public ClimberMoveCalibrate() {
    climber = RobotContainer.getSubsystem(Climber.class);
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    count = 0;
    climber.setArmVelocity(-Climber.ClimbCalibrateVel);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setArmVelocity(0.0);
    climber.setClimberPos(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (++count <= DELAY_COUNT) {
      return false;
    }
    return Math.abs(climber.getClimberVelocity()) < tolerance;
  }
}
