// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2024.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2024.subsystems.ShooterServo;

public class ShooterAngleSetPos extends Command {
  ShooterServo shooter;
  double desired_pos;
  boolean done;
 /* Move shooter to requested angle[deg] */
  public ShooterAngleSetPos(double desired_pos) {
    shooter = RobotContainer.getSubsystem(ShooterServo.class);
    this.desired_pos = desired_pos;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //shooter.setExtensionPosition(desired_pos); //we want  move , not calibraate
    shooter.setAngleSetpoint(desired_pos);
  }

  @Override
  public boolean isFinished() {
    return shooter.atSetpoint();
  }
}
