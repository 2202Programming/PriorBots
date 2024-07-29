// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2024.commands.Intake;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2024.subsystems.Intake;
import frc.robot2024.subsystems.Transfer;

/*
 * One button toggle based on current note location
 */
public class IntakeSwap extends Command {
  final Intake intake;
  final Transfer transfer;
  BooleanSupplier target;
  boolean checkIntake;
  int count;
  int DONE_COUNT;

  /** Creates a new IntakeSwitch. */
  public IntakeSwap() {
    intake = RobotContainer.getSubsystem(Intake.class);
    transfer = RobotContainer.getSubsystem(Transfer.class);
    addRequirements(intake, transfer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    count = 0;
    if (transfer.hasNote()) {
      target = intake::senseNote;
      DONE_COUNT = 10; // Delay for transfer --> intake
      transfer.setSpeed(-35.0);// [cm/s]
      intake.setIntakeSpeed(Intake.RollerEjectSpeed); // [cm/s] eject is neg
      checkIntake = true;
    } else if (intake.hasNote()) {
      target = transfer::senseNote;
      DONE_COUNT = 5; // Delay for intake --> transfer
      transfer.setSpeed(35.0); // [cm/s]
      intake.setIntakeSpeed(Intake.RollerMaxSpeed); // [cm/s]
      checkIntake = false;
    } else {
      // handle case where neither has note
      count = DONE_COUNT = 0;  // do nothing, signal done
      target = () -> {return false;};
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (target.getAsBoolean()) {
      count++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setIntakeSpeed(0.0);
    transfer.setSpeed(0.0);
    if(checkIntake){
      intake.setHasNote(true);
      transfer.setHasNote(false);
    }
    //if going into transfer, should all be done automatically
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return count >= DONE_COUNT;
  }
}