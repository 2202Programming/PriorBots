// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2026.command.pose;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2026.subsystems.LimelightV2;
import frc.robot2026.subsystems.Photonvision;
import frc.robot2026.subsystems.Photonvision.RobotCamera;
import frc.robot2026.subsystems.VisionPoseEstimator;

//This command should do a one-time gyro reset based on apriltags to align gyro with field heading.  
//Will do a low-accuracy single-tag estimate of rotation if that's all it can see, but will keep waiting for a multitag estimate before finally finishing.
//Should not need to be alliance-aware.

public class setGyroOffsetWithVision extends Command {
  /** Creates a new resetPoseWithVIsionAllianceAware. */

  private boolean singleResetDone;
  private boolean multiResetDone;
  private final VisionPoseEstimator vpe;
  private final Photonvision pv;
  private final LimelightV2 ll;

  public setGyroOffsetWithVision() {
    pv = RobotContainer.getSubsystemOrNull("photonvision");
    vpe = RobotContainer.getSubsystemOrNull("vision_odo");
    ll = RobotContainer.getSubsystemOrNull("limelight");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (ll != null) {
      ll.setUse_MT1(true); //MT1 should be used if no gyro offset done yet.
      ll.setUse_MT2(false);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //if this command has run before successfully, don't run again
    multiResetDone = vpe.hasGryoResetMultiTagHappened();
    singleResetDone = vpe.hasGryoResetSingleTagHappened();
    if (pv == null  && ll == null) {
      multiResetDone = true;
    }
    //dont update if already done or no vision
    if (multiResetDone) return;

    /*
     * Check each PV camera for one with multiple-tags, if found use it and declare
     * victory
     */
    if (pv != null && (pv.anyMultiTags())) { // someone has a multitag
      for (RobotCamera cam : pv.getCameras()) { 
        if (cam.hasMultitarget()) { // camera has a multitag, should be most reliable
          System.out.println("***Vision pose gryo Pre reset is: " + vpe.getPose().getRotation().getDegrees());
          Rotation2d tempRot = cam.getPose2d().getRotation();
          publish(tempRot);
          vpe.setGyroMultiTagDone();
          multiResetDone = true;
          System.out.println("***Vision pose gryo multitag reset done via PV, set to: "
                  + vpe.getPose().getRotation().getDegrees());
          return;
        }
      }
    }
    if (ll !=null && ll.getMT1Valid()) {
      if (ll.getMt1().tagCount > 1) { // Limelight has multitag
        System.out.println("***Vision pose gryo Pre reset is: " 
                          + vpe.getPose().getRotation().getDegrees());
        Rotation2d tempRot = ll.getMt1().pose.getRotation();
        publish(tempRot);
        vpe.setGyroMultiTagDone();
        multiResetDone = true;
        System.out.println("***Vision pose gryo multitag reset done via LL, set to: "
                          + vpe.getPose().getRotation().getDegrees());
        return;
      }
    }

    // got to here, no camera had 2 or more images.

    // there is no multitag yet, but one or more cameras have a single tag,
    // and we haven't done a single tag estimate yet
    if (pv != null && pv.totalTargetsAllCameras() > 0 && !singleResetDone) {
      System.out.println("***Vision pose gryo Pre reset is: " + vpe.getPose().getRotation().getDegrees());
      Rotation2d tempRot = pv.getAverageRot(); // there may be more than one camera with single tag, take an average of
                                               // their rotation estimates.
      publish(tempRot);
      vpe.setGyroSingleTagDone();
      singleResetDone = true;
      System.out
          .println(
              "***Vision pose gryo single tag reset done with PV, set to: " + vpe.getPose().getRotation().getDegrees());
    }
    if (ll !=null && ll.getMT1Valid() && !singleResetDone) {
      if (ll.getMt1().tagCount > 0) { // Limelight has one target
        System.out.println("***Vision pose gryo Pre reset is: " + vpe.getPose().getRotation().getDegrees());
        Rotation2d tempRot = ll.getMt1().pose.getRotation();
        publish(tempRot);
        vpe.setGyroSingleTagDone();
        singleResetDone = true;
        System.out
            .println(
                "***Vision pose gryo single tag reset done via LL, set to: "
                    + vpe.getPose().getRotation().getDegrees());
      }
    }
  }

  void publish(Rotation2d newRot) {
    if (vpe == null)
      return;
    vpe.setAnglePose(newRot);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (ll != null) {
      ll.setUse_MT1(false);
      ll.setUse_MT2(true); //back to MT2 for competition, presumably gyro offset has been set now.
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // End when we have a multi, or we get enabled.
    // OK to not be done in auto (it's using vision rot anyway).
    // Drivers should watch VPE gyro done flag to know if they need to hit Y to
    // manually do offset when tele starts.
    // TODO: set pose (and gyro offset) to start of auto path (where field team set up robot) if we get here an no gyro update done yet
    // probably means all vision is dorked up
    // like setPoseToPathStart()
    return multiResetDone || DriverStation.isTeleopEnabled();
  }

  @Override
  public boolean runsWhenDisabled() {
    return true; // Allows the command to run when disabled
  }
}
