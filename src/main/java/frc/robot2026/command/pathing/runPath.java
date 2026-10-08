// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot2026.command.pathing;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.subsystem.OdometryInterface;

public class runPath extends Command {
  
  Command pathCommand;
  final String pathName;
  final OdometryInterface odometry;

  public runPath(String pathName) {
    this("odometry", pathName);
  }

  public runPath(String odometryName, String pathName) {
    odometry = RobotContainer.getSubsystem(odometryName);
    this.pathName = pathName;
  }

  @Override
  public void initialize() {

    PathPlannerPath path;
    try {
      path = PathPlannerPath.fromPathFile(pathName);     
      pathCommand = AutoBuilder.followPath(path);

      CommandScheduler.getInstance().schedule(new InstantCommand(odometry::printPose));
      CommandScheduler.getInstance().schedule(pathCommand);
    
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      DriverStation.reportError("Big oops: No path cmd scheduled during initialize()", null);
    }
   }

  @Override
  public boolean isFinished() {
    return pathCommand.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("***RunPath Ended, current pose:");
    odometry.printPose();    
  }
}
