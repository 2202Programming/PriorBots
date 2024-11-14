/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.timbot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.timbot.subsystem.FlywheelSubsystem;
import frc.timbot.utils.Stick;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetSpinFlywheel extends InstantCommand {

  private FlywheelSubsystem m_flywheel;
  private double speed;

  public SetSpinFlywheel(FlywheelSubsystem flywheel, double speed) {
    m_flywheel = flywheel;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Stick.log("Starting SetSpinFlywheel2 command");
    m_flywheel.speed(speed);
  }
}
