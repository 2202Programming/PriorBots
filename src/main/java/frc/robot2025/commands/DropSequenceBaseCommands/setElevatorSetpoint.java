// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.commands.DropSequenceBaseCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.subsystem.UX.TrimTables.Trim;
import frc.robot2025.subsystems.Elevator_Subsystem;
import frc.robot2025.subsystems.Elevator_Subsystem.Levels;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class setElevatorSetpoint extends Command {
  Elevator_Subsystem elevator_Subsystem;
  Levels level;
  double setpoint;
  final Trim elevTrim;
  
  /** Creates a new setElevatorSetpoint. */
  public setElevatorSetpoint(Levels level, String name) {
    elevator_Subsystem = RobotContainer.getSubsystem(Elevator_Subsystem.class);
    this.level = level;
    setpoint = level.height;
    elevTrim = new Trim("Elevator", name);
    addRequirements(elevator_Subsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }
  public setElevatorSetpoint(double setpoint, String name) {
    elevator_Subsystem = RobotContainer.getSubsystem(Elevator_Subsystem.class);
    this.setpoint = setpoint;
    elevTrim = new Trim("Elevator",  name);
    addRequirements(elevator_Subsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator_Subsystem.setHeight(elevTrim.getValue(setpoint));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator_Subsystem.atSetpoint();
  }
}
