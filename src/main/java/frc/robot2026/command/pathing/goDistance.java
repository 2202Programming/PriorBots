// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2026.command.pathing;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.command.pathing.MoveToPose;
import frc.robot2026.subsystems.VisionPoseEstimator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class goDistance extends InstantCommand {

  private double distance;
  private Pose2d currentPose2d;
  private VisionPoseEstimator vpe;


  public goDistance(double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.distance = distance;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    vpe = RobotContainer.getSubsystemOrNull("vision_odo");
    currentPose2d = vpe.getPose();
    Pose2d targetPose2d = new Pose2d(currentPose2d.getX()+distance, currentPose2d.getY(),currentPose2d.getRotation());
    System.out.println("***Current pose: "+currentPose2d.getX()+","+currentPose2d.getY());
    System.out.println("***Target pose: "+targetPose2d.getX()+","+targetPose2d.getY());
    CommandScheduler.getInstance().schedule(new MoveToPose("vision_odo", targetPose2d));
  }
}
