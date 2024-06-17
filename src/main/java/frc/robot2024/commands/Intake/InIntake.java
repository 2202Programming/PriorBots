// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2024.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2024.subsystems.Intake;

/**
 * Driver presses button
 * Set intake angle to floor, turn on intake and transfer motors
 * Wait until lightgate detects note
 * Turn off intake
 * Wait x amount of time after lightgate detects note
 * shut off transfer motor, bring intake to movement pos, and turn blinky lights
 * to green!!
 * 
 */

public class InIntake extends Command {

  final Intake intake;
  final double DONE_COUNT = 2;
  double count;
  boolean stay_down;

  // State machine
  enum Phase {
    IntakeDown, WaitingForNote, Finished
  }

  Phase phase;

  /**
   * Intake will hold note in good position for it to deliver.
   * Assumes intake is empty and will pick up from the floor.
   */
  public InIntake(boolean stay_down) {
    this.stay_down = stay_down;
    this.intake = RobotContainer.getSubsystem(Intake.class);
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    count = 0;
    phase = Phase.IntakeDown;
    // intake.setHoldNote(true); // we want to keep the note
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (phase) {
      case IntakeDown:
        intake.setMaxVelocity(Intake.TravelDown);
        intake.setAngleSetpoint(Intake.DownPos);
        intake.setIntakeSpeed(Intake.RollerMaxSpeed); // [cm/s]
        phase = Phase.WaitingForNote;
        break;
      case WaitingForNote:
        // watch the intake State for note posession of Note
        if (intake.senseNote()) {
          count++;
        }
        if (count >= DONE_COUNT) {
          phase = Phase.Finished;
        }
        break;
      case Finished:
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setIntakeSpeed(0.0);
    if (!stay_down) {
      intake.setAngleVelocity(Intake.TravelUp);
      intake.setAngleSetpoint(Intake.UpPos);
    }
    if (intake.senseNote()) {
      intake.setHasNote(true);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return phase == Phase.Finished;
  }
}
