// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2025.subsystems.demo.CycloidalDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SpinCyclodialDrive extends Command {
  final CycloidalDrive drv;
  final double speed;

  /** Creates a new SpinCyclodialDrive. */
  public SpinCyclodialDrive(double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.speed = speed;
    // @Tyler, we don't want to create a new drive, we want to get the one constructed
    // during the robotContainer execution.
    //drv = new CycloidalDrive();
    drv = RobotContainer.getSubsystem(CycloidalDrive.class); //don't need name lookup if only one exists (common)
    addRequirements(drv);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drv.setVelocity(speed);
  }

  // Called once the command ends or is interrupted.  
  // This override can be deleted. - Mr. L
  @Override
  public void end(boolean interrupted) {
    // @Tyler - don't need this, it will stop right away.    //drv.setVelocity(0);
  }

  // Returns true when the command should end.
  // @Tyler - this command never ends, not what we really want.
  //          once the vel is set, this command is sort of done... and device will keep moving.
  @Override
  public boolean isFinished() {
    return true;   // @Tyler - changed from false set the speed in init and be done.
  }
}
