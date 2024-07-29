// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib2202.command.pathing;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.subsystem.swerve.SwerveDrivetrain;


//This command is a hack to reset our current pose to path start pose
//since we can't figure out how to make autobuilder do this
public class runPathResetStart extends Command {
  
  private Command pathCommand;
  final private SwerveDrivetrain drivetrain;

  public runPathResetStart() {
    drivetrain = RobotContainer.getSubsystem(SwerveDrivetrain.class);
  }

  @Override
  public void initialize() {

    PathPlannerPath path = PathPlannerPath.fromPathFile("test_1m");
    PathPoint startPoint = path.getPoint(0);
    Pose2d startPose = new Pose2d(
        new Translation2d(startPoint.position.getX(), startPoint.position.getY()),
        new Rotation2d(0.0)
    );

    pathCommand = AutoBuilder.followPath(path);

    drivetrain.autoPoseSet(startPose);
    new InstantCommand(drivetrain::printPose).schedule();
    pathCommand.schedule();
   }

  @Override
  public void execute() {

  }

  @Override
  public boolean isFinished() {
    return pathCommand.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("***RunPathResetStart Ended, current pose:");
    drivetrain.printPose();
    drivetrain.enableVisionPose();
  }
}
