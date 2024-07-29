// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib2202.subsystem.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.command.WatcherCmd;
import frc.lib2202.subsystem.swerve.config.ChassisConfig;

/*
 * Watcher for SwerveDrivetrain and its vision data.
 *
 *  Only watches high level data, for module details see the tables for each of the modules.
 */
public class DTMonitorCmd extends WatcherCmd {

  // Table Entries
  NetworkTableEntry currentX;
  NetworkTableEntry currentY;
  NetworkTableEntry currentHeading;

  NetworkTableEntry radiansPerSecond;
  NetworkTableEntry xMetersPerSec;
  NetworkTableEntry yMetersPerSec;

  NetworkTableEntry est_ll_pose_x;
  NetworkTableEntry est_ll_pose_y;
  NetworkTableEntry est_ll_pose_h;
  NetworkTableEntry est_pv_pose_x;
  NetworkTableEntry est_pv_pose_y;
  NetworkTableEntry est_pv_pose_h;

  // accessors for drivetrain
  final SwerveDrivetrain sdt;
  final ChassisConfig cc;
  final SwerveDriveOdometry odometry;

  Pose2d pose;
  Pose2d ll_pose;
  Pose2d pv_pose; 

  public DTMonitorCmd() {
    sdt = RobotContainer.getSubsystem(SwerveDrivetrain.class);
    odometry = sdt.getOdometry();
    cc = RobotContainer.getRobotSpecs().getChassisConfig();
    
    // use smartdashboard for complex objects
    var tname = getTableName();
    SmartDashboard.putData(tname + "/drive PIDF", cc.drivePIDF);
    SmartDashboard.putData(tname + "/angle PIDF", cc.anglePIDF);
  }

  @Override
  public String getTableName() {
    return SwerveDrivetrain.class.getSimpleName();
  }

  @Override
  public void ntcreate() {
    NetworkTable MonitorTable = getTable();

    currentX = MonitorTable.getEntry("bot_x");
    currentY = MonitorTable.getEntry("bot_y");
    currentHeading = MonitorTable.getEntry("bot_h");

    radiansPerSecond = MonitorTable.getEntry("vector w deg_p_sec");
    xMetersPerSec = MonitorTable.getEntry("vector x ");
    yMetersPerSec = MonitorTable.getEntry("vector y ");

    est_ll_pose_x = MonitorTable.getEntry("LL_x");
    est_ll_pose_y = MonitorTable.getEntry("LL_y");
    est_ll_pose_h = MonitorTable.getEntry("LL_h");

    est_pv_pose_x = MonitorTable.getEntry("PV_x");
    est_pv_pose_y = MonitorTable.getEntry("PV_y");
    est_pv_pose_h = MonitorTable.getEntry("PV_h");
  }

  @Override
  public void ntupdate() {
    // DriveTrain.drivePIDF should be handled via SmartDash buildable
    // read values from swerve drivetrain as needed using accesors
    pose = sdt.getPose();
    ll_pose = sdt.getLLEstimate();
    pv_pose = sdt.getPVEstimate();

    currentX.setDouble(pose.getX());
    currentY.setDouble(pose.getY());
    currentHeading.setDouble(pose.getRotation().getDegrees());

    // robot coordinates - speeds
    var speeds = sdt.getChassisSpeeds();
    radiansPerSecond.setDouble(speeds.omegaRadiansPerSecond * 57.3);
    xMetersPerSec.setDouble(speeds.vxMetersPerSecond);
    yMetersPerSec.setDouble(speeds.vyMetersPerSecond);

    if (ll_pose != null) {
      est_ll_pose_x.setDouble(ll_pose.getX());
      est_ll_pose_y.setDouble(ll_pose.getY());
      est_ll_pose_h.setDouble(ll_pose.getRotation().getDegrees());
    }
    if (pv_pose != null) {
      est_pv_pose_x.setDouble(pv_pose.getX());
      est_pv_pose_y.setDouble(pv_pose.getY());
      est_pv_pose_h.setDouble(pv_pose.getRotation().getDegrees());
    }

  }
}
