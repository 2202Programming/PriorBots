// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.chadbot.commands.test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.chadbot.subsystems.Magazine_Subsystem;
import frc.chadbot.subsystems.shooter.Shooter_Subsystem;

public class dumbshooter extends Command {
  /** Creates a new dumbshooter. */

  double upperTarget;
  double lowerTarget;
  double beltSpeed;
  Shooter_Subsystem shooter;
  Magazine_Subsystem magazine;

  public dumbshooter(Shooter_Subsystem shooter, Magazine_Subsystem magazine) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    this.shooter = shooter;
    this.magazine = magazine;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    upperTarget = 0;
    lowerTarget = 0;
    beltSpeed = 0;
    SmartDashboard.putNumber("UpperTarget%", upperTarget);
    SmartDashboard.putNumber("BeltSpeed%", beltSpeed);
    //SmartDashboard.putNumber("LowerTarget%", lowerTarget);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    upperTarget = SmartDashboard.getNumber("UpperTarget%", 0);
    //lowerTarget = SmartDashboard.getNumber("LowerTarget%", 0);
    beltSpeed = SmartDashboard.getNumber("BeltSpeed%", beltSpeed);
    shooter.onPercent(upperTarget, upperTarget);
    magazine.driveWheelOn(beltSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.off();
    magazine.driveWheelOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
