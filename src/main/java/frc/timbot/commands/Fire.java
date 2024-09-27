/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.timbot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.timbot.subsystems.Flywheel;
import frc.timbot.subsystems.Trigger;
import frc.timbot.utils.Stick;

public class Fire extends CommandBase {
  /**
   * Creates a new Fire.
   */
  private Trigger trigger;
  private Flywheel flywheel;

  public Fire(Trigger trigger, Flywheel flywheel) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.trigger = trigger;
    this.flywheel = flywheel;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Stick.log("Starting Fire command");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    trigger.fire();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (flywheel.upToSpeed()) {
    // return true;
    // }
    return true;
  }
}
