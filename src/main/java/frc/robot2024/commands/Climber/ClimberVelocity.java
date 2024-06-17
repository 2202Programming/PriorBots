// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2024.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2024.subsystems.Climber;

public class ClimberVelocity extends Command {
  Climber climber;
  double speed;

  /**
   * 
   * The arm will move the entire time this command is running, therefore
   * this command should be run are whileTrue() only
   * 
   * @param speed Arm movment vel, cm/s
   */
  public ClimberVelocity(double speed) {
    climber = RobotContainer.getSubsystem(Climber.class);
    this.speed = speed;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.setArmVelocity(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setArmVelocity(0.0);
  }
}
