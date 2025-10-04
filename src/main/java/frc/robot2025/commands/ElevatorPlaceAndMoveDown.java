// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib2202.builder.RobotContainer;
import frc.robot2025.subsystems.Elevator_Subsystem;
import frc.robot2025.subsystems.Elevator_Subsystem.Levels;
import frc.robot2025.subsystems.EndEffector_Subsystem;
import frc.robot2025.subsystems.Sensors_Subsystem;
import frc.robot2025.subsystems.WristFLA;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ElevatorPlaceAndMoveDown extends SequentialCommandGroup {

  Elevator_Subsystem elevator;
  EndEffector_Subsystem endEffector;
  Sensors_Subsystem sensors;
  WristFLA wrist;
  Levels setPoint;

  /** Creates a new ElevatorPlaceAndMoveDown2. */
  public ElevatorPlaceAndMoveDown(Levels setPoint) {
    elevator = RobotContainer.getSubsystem(Elevator_Subsystem.class);
    endEffector = RobotContainer.getSubsystem("endEffectorSubsystem");
    sensors = RobotContainer.getSubsystem("sensors");
    wrist = RobotContainer.getSubsystem(WristFLA.class);

    this.setPoint = setPoint;

    if (!endEffector.pieceReady()) {
      System.out.println("No piece found");
      end(true);
    } else {
      addCommands(new 
        WristFLAToPos(WristFLA.PICKUP_POSITION)
        .andThen(new ElevatorMove(setPoint))
        .andThen(new EndEffectorRPM(1000))
        .andThen(new WaitCommand(0.5))
        );

      if (!endEffector.pieceReady()) {
        addCommands(new
        EndEffectorRPM(0)
        .andThen(new ElevatorMove(Levels.PickUp))
        .andThen(new WristFLAToPos(WristFLA.DROP_POSITION))
        );
      } else {
        System.out.println("******ERROR: Piece still found");
        end(true);
      }
    }


    addRequirements(this.elevator);
    addRequirements(this.endEffector);
  }
}
