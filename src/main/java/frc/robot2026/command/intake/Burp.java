// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2026.command.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2026.subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Burp extends Command {

  final Intake intake;
  double targetPos;
  boolean forward;

  double backRot;
  double forwardRot;

  public Burp(double backRot, double forwardRot) {
    this.backRot = backRot;
    this.forwardRot = forwardRot;

    intake = RobotContainer.getSubsystem("intake");
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    forward = false;
    intake.zeroPos();
    targetPos = backRot;
    intake.setPosSetpoint(targetPos);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(intake.getPosition() - targetPos) < 0.03) {
      targetPos = forwardRot;
      intake.setPosSetpoint(targetPos);
      forward = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setPercent(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return forward && (Math.abs(intake.getPosition() - targetPos) < 0.03);
  }
}
