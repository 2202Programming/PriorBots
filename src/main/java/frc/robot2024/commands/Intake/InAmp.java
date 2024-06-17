// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2024.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2024.subsystems.Intake;

public class InAmp extends Command {
  final Intake intake;
  final int DONE_COUNT = 20; //placeholder
  double count;
  /** Creates a new InAmp. */
  public InAmp() {
    this.intake = RobotContainer.getSubsystem(Intake.class);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setMaxVelocity(20.0);
    intake.setAngleSetpoint(30.0); //placeholder - find value that works
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(intake.angleAtSetpoint()){
      intake.setIntakeSpeed(Intake.RollerAmpSpeed); //[cm/s]
    }
    if(intake.rollerAtVelocity()){
      count++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //add max vel?
    intake.setMaxVelocity(120.0);
    intake.setAngleSetpoint(0.0);
    intake.setIntakeSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return count >= DONE_COUNT;
  }
}
