/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.timbot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.timbot.subsystems.Trigger;
import frc.timbot.utils.Stick;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Idle extends InstantCommand {
  private Trigger trigger;

  public Idle(Trigger trigger) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.trigger = trigger;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Stick.log("Starting Idle command");
    trigger.idle();
  }
}
