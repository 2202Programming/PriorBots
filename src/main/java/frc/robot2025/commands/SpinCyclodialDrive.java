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
  final double value;
  final boolean use_pos_mode; //True Position False Velocity

  public SpinCyclodialDrive(double speed, boolean use_pos_mode) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.value = speed;
    this.use_pos_mode = use_pos_mode;

    drv = RobotContainer.getSubsystem(CycloidalDrive.class); //don't need name lookup if only one exists (common)
    addRequirements(drv);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(use_pos_mode)
    {
      drv.setSetpoint(value);
    }else{
      drv.setVelocity(value);
    }
  }

  @Override
  public boolean isFinished() {
    // deal with each mode
    if (use_pos_mode == false) {
      return true;   // we are done after the vel is set, device will start spinning
    }
    
    //using position mode, not done until we get to setpoint
    return drv.atSetpoint();
  }
}
