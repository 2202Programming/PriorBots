
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2024.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2024.subsystems.ShooterServo;

public class CalibrateAngle extends Command {

  ShooterServo shooter;
  double vel;
  boolean done;
  /** Creates a new ShooterAngleVelMove. */
  public CalibrateAngle(double vel) {
    shooter = RobotContainer.getSubsystem(ShooterServo.class);
    this.vel = vel;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    done = false;
    shooter.setExtensionVelocity(vel);
  }

  @Override
  public void execute(){
    if(shooter.atHighLimit() || shooter.atLowLimit()){
        done = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(shooter.atHighLimit()){
    shooter.setExtensionPosition(0.0); // [cm] - find how much off from actual zeor
    }
    else if(shooter.atLowLimit()){
        shooter.setExtensionPosition(0.0); //[cm] - find how much off from actual zero
    }
    shooter.setExtensionVelocity(0.0); // [cm/s]
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}

