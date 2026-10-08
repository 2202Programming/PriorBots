// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2026.command.pathing;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib2202.subsystem.OdometryInterface;

//Set odometery pose to starting pose of path, if needed.
public class setPoseToPathStart extends InstantCommand {
  
  private PathPlannerPath path;
  private OdometryInterface odo;

  public setPoseToPathStart(OdometryInterface odo, PathPlannerPath path) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.path = path;
    this.odo = odo;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d startingPose = path.getStartingHolonomicPose().get();
    odo.setPose(startingPose);
  }
}
