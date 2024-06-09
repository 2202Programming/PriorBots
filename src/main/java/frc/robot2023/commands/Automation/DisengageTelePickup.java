// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2023.commands.Automation;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.base.RobotContainerOrig;
import frc.robot2023.commands.Arm.ArmLockForDrivingFS;
import frc.robot2023.commands.auto.moveToPoint;

public class DisengageTelePickup extends SequentialCommandGroup {
  /** Creates a new PlaceHighAuto. */
 
  public DisengageTelePickup() {
    Pose2d targetPose;
    Pose2d currentPose = RobotContainerOrig.RC().drivetrain.getPose();

    if(DriverStation.getAlliance() == DriverStation.Alliance.Blue) { //BLUE ALLIANCE
      targetPose = new Pose2d(currentPose.getX() - 0.2, currentPose.getY(), currentPose.getRotation()); //20 cm away from scoring station, blue side
    }
    else{
      targetPose = new Pose2d(currentPose.getX() + 0.2, currentPose.getY(), currentPose.getRotation()); //20 cm away from scoring station, red side
    }

   addCommands(
        new ParallelCommandGroup(
            new moveToPoint(new PathConstraints(0.2, 0.1), targetPose),
            new ArmLockForDrivingFS()
        )
   );
  }
}
