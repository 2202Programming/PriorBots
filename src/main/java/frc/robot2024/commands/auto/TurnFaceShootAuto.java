// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2024.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.command.swerve.FaceToTag;
import frc.lib2202.command.swerve.RotateUntilSeeTags;
import frc.lib2202.subsystem.Limelight;
import frc.lib2202.subsystem.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot2024.Constants.Tag_Pose;
import frc.robot2024.commands.Shooter.ShooterSequence;


public class TurnFaceShootAuto extends Command {
  int tagID;
  Limelight limelight;

  public TurnFaceShootAuto(int tagID) {
    this.tagID = tagID;
    limelight = RobotContainer.getSubsystem(Limelight.class);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      if(checkForTarget()) { //target tag is visible
        new SequentialCommandGroup(
          new FaceToTag(tagID),
          new ShooterSequence(3000)
        ).schedule();
      } 
      else{ //target tag is not *yet* visible
        new SequentialCommandGroup(
          new RotateUntilSeeTags(Tag_Pose.ID4, Tag_Pose.ID7),
          new FaceToTag(tagID),
          new ShooterSequence(3000)
        ).schedule();
      }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }

  /**
   * Check if limelight can see the target.
   * 
   * @param tagID tagID to check for in limelight
   * @return {@code true} if the target is found in limelight, {@code false} if
   *         not.
   */
  private boolean checkForTarget() {
    LimelightTarget_Fiducial[] tags = limelight.getAprilTagsFromHelper();
    for (LimelightTarget_Fiducial tag : tags) {
      if ((int)tag.fiducialID == tagID) {
        return true;
      }
    }
    return false;
  }

}
