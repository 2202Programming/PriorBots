/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.timbot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.timbot.subsystems.Flywheel;
import frc.timbot.subsystems.Trigger;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FireThenIdle extends SequentialCommandGroup {
  /**
   * Creates a new FireThenIdle.
   */
  public FireThenIdle(Trigger trigger, Flywheel flywheel) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new Fire(trigger, flywheel),

        new WaitCommand(0.1),

        new Idle(trigger),

        new WaitCommand(1.0)

    );
  }
}
