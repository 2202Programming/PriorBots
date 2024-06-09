// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2023.commands.Automation;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.base.RobotContainerOrig;
import frc.robot2023.commands.Arm.ArmLockForDrivingBS;
import frc.robot2023.commands.Arm.ArmLockForDrivingFS;
import frc.robot2023.commands.auto.moveToPoint;

public class DisengageTelePlace extends SequentialCommandGroup {

  /**
   * Constructs a DisengageTelePlace
   * @param constraints path constraints
   * @param distance distance to creep away from scoring station to make room for arm retraction (meters)
   */
   public DisengageTelePlace(PathConstraints pathConstraints, Pose2d targetPose) {

   addCommands(
            new moveToPoint(pathConstraints, targetPose),
            new ArmLockForDrivingFS()
        );
  }
}
