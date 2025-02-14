// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.chadbot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.subsystem.swerve.SwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ResetPosition extends InstantCommand {
  private Pose2d newPose;
  private SwerveDrivetrain m_drivetrain;
  
  public ResetPosition(Pose2d newPose) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.newPose = newPose;
    this.m_drivetrain = RobotContainer.getSubsystem(SwerveDrivetrain.class);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.setPose(newPose);
  }
}
