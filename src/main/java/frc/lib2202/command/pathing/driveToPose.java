// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib2202.command.pathing;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.subsystem.swerve.SwerveDrivetrain;

public class driveToPose extends Command {

  private Pose2d targetPose;
  private SwerveDrivetrain m_Drivetrain;
  private PathConstraints pathConstraints;

  /** Creates a new driveToPose. */
  public driveToPose(Pose2d targetPose) {

    this.targetPose = targetPose;
    m_Drivetrain = RobotContainer.getSubsystem("DRIVETRAIN");
    addRequirements(m_Drivetrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pathConstraints = new PathConstraints(1.0, 1.0, Units.degreesToRadians(360), Units.degreesToRadians(540));

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    AutoBuilder.pathfindToPose(targetPose, pathConstraints, 0.0).schedule();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
