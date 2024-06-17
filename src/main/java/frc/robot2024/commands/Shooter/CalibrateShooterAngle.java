
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2024.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2024.subsystems.ShooterServo;

public class CalibrateShooterAngle extends Command {
  final double VEL_ZERO_LIMIT = 0.2; //[cm/s]
  final int DELAY_COUNT = 5; // 0.1 sec to let it start moving before checking

  ShooterServo shooter;
  double vel;
  int count;
  
  /** Creates a new ShooterAngleVelMove. */
  public CalibrateShooterAngle(double vel) {
    shooter = RobotContainer.getSubsystem(ShooterServo.class);
    this.vel = vel;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setExtensionVelocity(vel);
    count = 0;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setExtensionPosition(0.0); // [cm]
    shooter.setExtensionVelocity(0.0); // [cm/s]
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (++count <= DELAY_COUNT) {
      return false;
    }
    // watch for it to stop moving
    return (Math.abs(shooter.getExtensionVelocity()) <= VEL_ZERO_LIMIT);
  }
}

