// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2024.commands.Intake;

// TODO - Mr.L I don't think we need this with the external encoder
//  leaving for now

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2024.subsystems.Intake;

@Deprecated  // likely won't have ABS Encoder
public class AbsEncoderCalibrate extends Command {
  final Intake intake;
  /** Creates a new AbsEncoderCalibrate. */
  public AbsEncoderCalibrate() {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = RobotContainer.getSubsystem(Intake.class);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //intake.calibratePos();  //I don't think this is what you really wanted.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.angleAtSetpoint();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
