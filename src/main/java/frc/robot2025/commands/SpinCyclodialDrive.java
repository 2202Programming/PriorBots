// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot2025.subsystems.CycloidalDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SpinCyclodialDrive extends Command {
  CycloidalDrive drv;
  double speed;

  /** Creates a new SpinCyclodialDrive. */
  public SpinCyclodialDrive(double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.speed = speed;
    drv = new CycloidalDrive();
    addRequirements(drv);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drv.setVelocity(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drv.setVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
