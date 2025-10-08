// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.subsystems;

import static frc.lib2202.Constants.DEGperRAD;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.subsystem.BaseLimelight;
import frc.lib2202.subsystem.LimelightHelpers;
import frc.lib2202.subsystem.swerve.IHeadingProvider;
import frc.robot2025.Constants.TheField;

public class Limelight extends BaseLimelight {
 
  //access to mt2 complete object
  LimelightHelpers.PoseEstimate mt2;
  protected Pose2d bluePose2 = new Pose2d();
  protected final IHeadingProvider gyro;
  protected final int[] allTags;

  public Limelight() {
    this("limelight");
  }

  public Limelight(String name) {
    this(name, new Pose3d()); // 0,0,0... not great
  }

  public Limelight(String name, Pose3d position) {
    super(name);
    gyro = RobotContainer.getRobotSpecs().getHeadingProvider();

    // Set our camera's position on the robot
    var llRot = position.getRotation();
    LimelightHelpers.setCameraPose_RobotSpace(name,
        position.getX(), position.getY(), position.getZ(),
        llRot.getX() * DEGperRAD, llRot.getY() * DEGperRAD, llRot.getZ() * DEGperRAD);

    LimelightHelpers.SetIMUMode(name, 1); //fused our gyro with theirs -LL4

    // save all the tags for reseting LL tag filter.
    var tgs = TheField.fieldLayout.getTags();
    allTags = new int[tgs.size()];
    for (int i = 0; i < tgs.size(); i++)
      allTags[i] = tgs.get(i).ID;
  }

  // Some helper methods not in the base, candidates for moving to baseclass, but
  // may be LL4
  public void setTargetTags(int[] tags) {
    LimelightHelpers.SetFiducialIDFiltersOverride(name, tags);
  }

  public void setTargetTagsAll() {
    LimelightHelpers.SetFiducialIDFiltersOverride(name, allTags);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pipeline = nt_pipelineNTE.getInteger(0);

    LimelightHelpers.SetRobotOrientation(name, gyro.getHeading().getDegrees(), 0, 0, 0, 0, 0);
    // LimelightHelpers.PoseEstimate mt1 =
    // LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
    mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
    
    //outputs 
    targetValid = false;
    rejectUpdate = true;
    numAprilTags = 0; // gets changed if mt2 is valid
    // in sim, mt2 is null, need to protect so we can debug
    if (mt2 == null){
      log();  //update NT with values
      return;
    }

    // rejectUpdate on poor conditions @Dr.J ?? what does [0] refer too???
    rejectUpdate = 
        //(mt2.rawFiducials[0].ambiguity > 0.7) ||     //mt1 only
        //(mt2.rawFiducials[0].distToCamera > 3.0) ||  //mt1 only
        (mt2.tagCount == 0) ||
        (mt2.pose.getX() == 0.0) ||            // rejest if 0,0,0 pose - chief delphi rec
        (Math.abs(gyro.getYawRate()) > 720.0); // reject if spining fast

    if (!rejectUpdate) {
      bluePose = mt2.pose;
    }
    numAprilTags = mt2.tagCount;

    // set output vars for accessors
    targetValid = (numAprilTags > 0);
    visionTimestamp = Timer.getFPGATimestamp()
        - (LimelightHelpers.getLatency_Pipeline(this.name) / 1000.0)
        - (LimelightHelpers.getLatency_Capture(this.name) / 1000.0);

    log(); // do logging at end
  }

  public LimelightHelpers.PoseEstimate getMt2() {
    return mt2;
  }
}
