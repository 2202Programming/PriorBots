// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2024.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.base.RobotContainerOrig;
import frc.robot2024.commands.Shooter.ShooterSequence;
import frc.robot2024.commands.Swerve.FaceToTag;
import frc.robot2024.commands.Swerve.RotateUntilSeeTags;
import frc.robot2024.subsystems.Sensors.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot2024.subsystems.Sensors.Limelight_Subsystem;


public class TurnFaceShootAuto extends Command {
  double targetTagID;
  int tagID;
  Limelight_Subsystem limelight;

  public TurnFaceShootAuto(int tagID) {
    targetTagID = tagID;
    this.tagID = tagID;
    limelight = RobotContainerOrig.getSubsystem(Limelight_Subsystem.class);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      if(checkForTarget()) { //target tag is visible
        new SequentialCommandGroup(
          new FaceToTag(targetTagID),
          new ShooterSequence(3000)
        ).schedule();
      } 
      else{ //target tag is not *yet* visible
        new SequentialCommandGroup(
          new RotateUntilSeeTags(),
          new FaceToTag(targetTagID),
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
      if (tag.fiducialID == targetTagID) {
        return true;
      }
    }
    return false;
  }

}
