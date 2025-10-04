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
public class DropSequence extends Command {
  /** Creates a new DropSequence. */
  final Elevator_Subsystem elevator;
  final WristFLA wrist;
  int DELAY_COUNT = 25;
  int count;
  final EndEffector_Subsystem endEffector;
  Levels level;
  public enum Phase{
    ElevatorUp, Drop, Finish
  }

  Phase phase;
  public DropSequence(Levels level) {
    elevator = RobotContainer.getSubsystem(Elevator_Subsystem.class);
    wrist = RobotContainer.getSubsystem(WristFLA.class);
    endEffector = RobotContainer.getSubsystem(EndEffector_Subsystem.class);
    this.level = level;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    count = 0;
    phase = Phase.ElevatorUp;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(phase){
      case ElevatorUp:
        elevator.setHeight(level);
        wrist.setPosition(WristFLA.DROP_POSITION);
        //next state conditions
        if(elevator.atSetpoint() && wrist.atSetpoint()){
          elevator.setVelocity(0);
          phase = Phase.Drop;
        }
        break;
      case Drop:
      if(count == 0){

        //Mr.L do we KNOW we are ready to drop?  Drivetrain still moving? Aligned?
        // any how, good first cut.
        endEffector.setRPM(500.0); //placeholder
      }
      if(++count >= DELAY_COUNT){
        endEffector.setRPM(0);
        phase = Phase.Finish;
      }
       break;
      default:
        break; 
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Mr.L - do we want to start the elevator/wrist moving down right away???
    // Does the drivetrain have to backup any?
    elevator.setHeight(0); //pickup level
    wrist.setPosition(WristFLA.PICKUP_POSITION); //default pos
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return phase == Phase.Finish;
  }
}
