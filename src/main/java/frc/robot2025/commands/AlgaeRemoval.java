// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2025.subsystems.Elevator_Subsystem;
import frc.robot2025.subsystems.EndEffector_Subsystem;
import frc.robot2025.subsystems.WristFLA;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeRemoval extends Command {
  /** Creates a new AlgaeRemoval. */
  Elevator_Subsystem elevator_Subsystem;
  EndEffector_Subsystem ee_Subsystem;
  WristFLA wrist; 
  public AlgaeRemoval() {
    elevator_Subsystem = RobotContainer.getSubsystem(Elevator_Subsystem.class);
    ee_Subsystem = RobotContainer.getSubsystem(EndEffector_Subsystem.class);
    wrist = RobotContainer.getSubsystem(WristFLA.class);
    addRequirements(elevator_Subsystem, ee_Subsystem, wrist);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wrist.setPosition(WristFLA.ALGAE_REMOVAL_POSITION);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(wrist.atSetpoint()){
    elevator_Subsystem.setVelocity(60.0);
    ee_Subsystem.setPercent(-0.6);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator_Subsystem.setHeight(0.0);
    ee_Subsystem.setPercent(0.0);
    wrist.setPosition(WristFLA.PICKUP_POSITION);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
