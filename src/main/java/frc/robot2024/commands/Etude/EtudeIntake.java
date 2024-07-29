// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2024.commands.Etude;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib2202.builder.RobotContainer;
import frc.robot2024.commands.Intake.MoveToAnglePos;
import frc.robot2024.subsystems.Intake;
import frc.robot2024.subsystems.Shooter;
import frc.robot2024.subsystems.Transfer;

/**
 * Driver presses button
 * Set intake angle to floor, turn on intake and transfer motors
 * Wait until lightgate detects note
 * Turn off intake
 * Wait x amount of time after lightgate detects note
 * shut off transfer motor, bring intake to movement pos, and turn blinky lights
 * to green!!
 */

public class EtudeIntake extends Command {

  final Intake intake;
  final Transfer transfer;
  final Shooter shooter;
  boolean stay_down;
  boolean pneumatics_bot = false;
  int count;
  final int DONE_COUNT = 15;
  boolean sensed_note;
  final double ELVIS_INTKAE_DOWN_ANGLE = 91.0;

  public enum Phase {
    IntakeDown, WaitingForNote, Finished, HaveNote
  }

  Phase phase;

  /*
   * stay_down = true for No defense rapid shoot
   */
  public EtudeIntake(boolean stay_down) {
    this.stay_down = stay_down;
    this.intake = RobotContainer.getSubsystem(Intake.class);
    this.transfer = RobotContainer.getSubsystem(Transfer.class);
    this.shooter = RobotContainer.getSubsystemOrNull("SHOOTER");
    addRequirements(intake, transfer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sensed_note = false;
    count = 0;
    phase = Phase.IntakeDown;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (phase) {
      case IntakeDown:
        if (shooter != null)
          shooter.retract();
        intake.setMaxVelocity(Intake.TravelDown);
        intake.setAngleSetpoint(ELVIS_INTKAE_DOWN_ANGLE);
        intake.setIntakeSpeed(0.8); // %
        transfer.setSpeed(40.0);
        phase = Phase.WaitingForNote;
        break;
      case WaitingForNote:
        phase = transfer.hasNote() ? Phase.HaveNote : Phase.WaitingForNote;
        break;
      case HaveNote:
        if (++count >= DONE_COUNT) {
          intake.setMaxVelocity(Intake.TravelUp);
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
    if (interrupted) {
      // Creates a command to continue going down until we get to the bottom before
      // moving back up, to minimize belt slippage
      System.out.println("Interrupted intakeSequence");
      var cmd = new SequentialCommandGroup();
      if (!intake.angleAtSetpoint()) {
        cmd.addCommands(new MoveToAnglePos(ELVIS_INTKAE_DOWN_ANGLE, Intake.TravelDown));
      }
      cmd.addCommands(new MoveToAnglePos(Intake.UpPos, Intake.TravelUp));
      cmd.addRequirements(intake);
      cmd.schedule();
    }

    if (!stay_down && !interrupted) {
      intake.setMaxVelocity(Intake.TravelUp);
      intake.setAngleSetpoint(Intake.UpPos);
    }
    transfer.setSpeed(0.0);
    intake.setIntakeSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return phase == Phase.Finished;

  }
}