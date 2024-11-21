/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.timbot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.timbot.subsystems.Elevation;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SendToMax extends InstantCommand {

  private Elevation m_elevator; //works entirely in cm

  public SendToMax(Elevation m_elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_elevator = m_elevator;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_elevator.getPos() > 5.9) {
      m_elevator.setPoint(0);
    } else {
      m_elevator.setPoint(6);
    }
  } 

  public void execute() {

  }

  public void end() {
    m_elevator.setPoint(m_elevator.getPos());
  }

  public boolean isFinished() {
    return m_elevator.isAtPosition();
  }

}

