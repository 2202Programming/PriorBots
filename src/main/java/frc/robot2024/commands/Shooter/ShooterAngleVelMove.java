// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2024.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2024.subsystems.ShooterServo;

public class ShooterAngleVelMove extends Command {
  ShooterServo shooter;
  double vel;
  int count;
  final int DONE_COUNT = 500;

  /** Creates a new ShooterAngleCalibrate. */
  public ShooterAngleVelMove(double vel) {
    this.vel = vel;
    shooter = RobotContainer.getSubsystem(ShooterServo.class);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    count = 0;
    shooter.setExtensionVelocity(vel);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    count = (Math.abs(shooter.getExtensionVelocity()) < 0.1) ? ++count : 0;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Shooter Extension velcmd ENDING FOR NO REASON***************************************");
    shooter.setExtensionVelocity(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return count >= DONE_COUNT;
  }
}