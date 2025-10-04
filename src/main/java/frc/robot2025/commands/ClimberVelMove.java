// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2025.subsystems.Climber;

public class ClimberVelMove extends Command {
  /** Creates a new ClimberSetPos. */
  final Climber climber;
  double speed;
  public ClimberVelMove(double speed) {
    climber = RobotContainer.getSubsystem(Climber.class);
    this.speed = speed;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.setIAccum(0.0);  // clear any windup
    climber.setVelocity(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setIAccum(0.0); // prevent integrator being left wound up
    climber.setVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // cmd runs until driver releases button
  }

}