// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2024.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2024.subsystems.Transfer;
/*
 * Simple command to pull note through transfer and stop at good Note location.
 */
public class TestTransfer extends Command {
  final Transfer transfer;
  final double speed; 
  final int MAX_COUNT = 1;
  int count =0;

  public TestTransfer(double speed) {
    this.transfer = RobotContainer.getSubsystem(Transfer.class);
    this.speed = speed;
    addRequirements(transfer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    transfer.setHasNote(false);
    transfer.setSpeed(speed);
    count = 0;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    transfer.setSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if we are loading note, speed > 0, then we can stop on count
    if (speed > 0.0 && transfer.hasNote()) return (++count >= MAX_COUNT);
    return false;
  }
}
