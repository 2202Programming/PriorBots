/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Elevation;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AdjustElevation extends InstantCommand {

  private Elevation m_elevator;
  private double angle;

  public AdjustElevation(Elevation m_elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_elevator = m_elevator;
    angle = 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angle = angle + 50;
    m_elevator.setActuatorAngle(angle);
  }
}
