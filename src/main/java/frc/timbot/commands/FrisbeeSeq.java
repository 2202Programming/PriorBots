// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.timbot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib2202.builder.RobotContainer;
import frc.timbot.subsystem.Trigger;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FrisbeeSeq extends SequentialCommandGroup {
  /** Creates a new FrisbeeSeq. */

  final Trigger trigger;

  public FrisbeeSeq() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    trigger = RobotContainer.getSubsystem(Trigger.class);

    addCommands(trigger.trigger_fire(), new WaitCommand(1), trigger.trigger_reset());

    
  }
}
