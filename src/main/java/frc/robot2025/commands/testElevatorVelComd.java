// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2025.subsystems.Elevator_Subsystem;

public class testElevatorVelComd extends Command {
  /** Creates a new testElevatorVelComd. */
   //should be done as a while true command 
  private final Elevator_Subsystem elevator_Subsystem;
  double vel;
  
  public testElevatorVelComd(double vel) {
    SmartDashboard.putNumber("Current Vel", 30.0);
    elevator_Subsystem = RobotContainer.getSubsystem(Elevator_Subsystem.class);
    this.vel = vel;
    // Use addRequirements() here to declare subsystem dependencies.
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    vel = SmartDashboard.getNumber("Current Vel", 30.0);
    elevator_Subsystem.setVelocity(vel);
    System.out.println(vel + "CURRENT PRINT");
    System.out.println(elevator_Subsystem.getDesiredVelocity() + "Desired vel");
    System.out.println(elevator_Subsystem.getVelocity() + "Current");
  }

  @Override
  public void execute(){
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println(elevator_Subsystem.getVelocity() + "released");
    elevator_Subsystem.setVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
