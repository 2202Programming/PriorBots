// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2024.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2024.subsystems.Climber;

public class Climb extends Command {
  /** Creates a new Climber. */
  private Climber climber;
  private double pos;
  /** 
   * Climb will just set both arms to a single position. It does nothing more than that.
   * 
   * @param pos [cm] position that we want BOTH arms to go to.
   */
  public Climb(double pos) {  
    // Use addRequirements() here to declare subsystem dependencies.
    climber = RobotContainer.getSubsystem(Climber.class);
    this.pos = pos;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.setSetpoint(pos);
  }

  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return  climber.atSetpoint();    
    //note: climber will remain running, holding the last setpoint
  }
}