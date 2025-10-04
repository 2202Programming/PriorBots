// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot2025.subsystems.Elevator_Subsystem;
import frc.robot2025.subsystems.EndEffector_Subsystem;
import frc.lib2202.builder.RobotContainer;
import frc.robot2025.subsystems.Elevator_Subsystem.Levels;
import frc.robot2025.subsystems.WristFLA;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PickupSequence extends Command {
  Elevator_Subsystem elevator;
  WristFLA wrist;
  EndEffector_Subsystem endEffector;
  Levels level;
  boolean station;
  final int DELAY_COUNT = 25;
  int count;

  /** Creates a new PickupSequence. */
  public enum Phase {
    ElevatorInPos, WaitingForCoral, Finished
  }

  Phase phase;
  /**
   * 
   * @param level what level you want to go after pickup
   * @param station whether pickup is from the station or no
   */

  public PickupSequence(Levels level, boolean station) {
    elevator = RobotContainer.getSubsystem(Elevator_Subsystem.class);
    wrist = RobotContainer.getSubsystem(WristFLA.class);
    endEffector = RobotContainer.getSubsystem(EndEffector_Subsystem.class);
    this.station = station;
    this.level = level;
    // Use addRequirements() here to declare subsystem dependencies.
    //systwem.out.println("sup, from Avdhut");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    phase = Phase.ElevatorInPos;
    count = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(phase){
      case ElevatorInPos:
      if(station){
        elevator.setHeight(Levels.LTwo);
      } else{
        elevator.setHeight(Levels.LOne);
      }
        wrist.setPosition(WristFLA.DROP_POSITION);
        if(elevator.atSetpoint() && wrist.atSetpoint()){
          elevator.setVelocity(0);
          phase = Phase.WaitingForCoral;
        }
        break;
      case WaitingForCoral:
      if(station){
        endEffector.setRPM(500); //placeholder
        if(endEffector.pieceReady()){
          count++;
        }
      } else {
        endEffector.setRPM(-500); //placeholder
        if(endEffector.hasPiece()){
          count++;
        }
      }
        if(count >= DELAY_COUNT){
          phase = Phase.Finished;
        }
        break;
      default:
      break;
        }
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrist.setPosition(WristFLA.DROP_POSITION);
    elevator.setHeight(Levels.LFour);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return phase == Phase.Finished;
  }
}
