// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2025.subsystems.WristFLA;

public class SDWrist extends Command {
  /** Creates a new testElevatorVelComd. */
   //should be done as a while true command 
  private final WristFLA wrist;
  double pos;
  
  public SDWrist(double pos) {
    SmartDashboard.putNumber("DesPosWrist", 0.0);
    wrist = RobotContainer.getSubsystem(WristFLA.class);
    this.pos = pos;
    // Use addRequirements() here to declare subsystem dependencies.
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pos = SmartDashboard.getNumber("DesPosWrist", 0.0);
    wrist.setPosition(pos);
    System.out.println(pos + "WRIST PRINT");
  }

  @Override
  public void execute(){
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return wrist.atSetpoint();
  }
}
