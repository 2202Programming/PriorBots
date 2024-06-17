// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2024.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2024.subsystems.Intake;
import frc.robot2024.subsystems.Transfer;

public class EjectNote extends Command {
  /** Creates a new IntakeReverse. */
  final Intake intake;
  final Transfer transfer;
  public EjectNote() {
    this.intake = RobotContainer.getSubsystem(Intake.class);
    this.transfer = RobotContainer.getSubsystem(Transfer.class);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setMaxVelocity(60.0);
    intake.setAngleSetpoint(100.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(intake.getAnglePosition() >50.0){
    intake.setIntakeSpeed(Intake.RollerEjectSpeed);
    transfer.setSpeed(-35.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    transfer.setHasNote(false);
    intake.setMaxVelocity(120.0);
    intake.setAngleSetpoint(0.0);
    intake.setIntakeSpeed(0.0);
    transfer.setSpeed(0.0);
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
