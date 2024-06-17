// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib2202.command.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.subsystem.Limelight;
import frc.lib2202.subsystem.LimelightHelpers.LimelightTarget_Fiducial;
import frc.lib2202.subsystem.swerve.SwerveDrivetrain;

public class RotateUntilSeeTags extends Command {
  final SwerveDrivetrain drivetrain;
  Limelight limelight;

  private PIDController pid;
  private final double kp = 0.04;
  private final double ki = 0.0;
  private final double kd = 0.0;
  private final double pos_tol = 5.0;
  private final double vel_tol = 1.0;

  private SwerveDriveKinematics kinematics;
  private Pose2d currentPose;
  private double targetRot;
  private SwerveModuleState[] outputModuleState;

  Translation2d targetPose; // Position want to face to
  int targetID;

  private Timer timer;

  //Alliance 
  final Translation2d redTarget;
  final int redID;
  final Translation2d blueTarget;
  final int blueID;

  /** Creates a new RotateTo. */
  public RotateUntilSeeTags(int redID, Translation2d redTarget, int blueID, Translation2d blueTarget) {
    this.redID = redID;
    this.redTarget = redTarget;
    this.blueID = blueID;
    this.blueTarget = blueTarget;

    drivetrain = RobotContainer.getSubsystem(SwerveDrivetrain.class);
    limelight = RobotContainer.getSubsystem(Limelight.class);
    addRequirements(drivetrain);
    pid = new PIDController(kp, ki, kd);
    pid.enableContinuousInput(-180.0, 180.0);
    pid.setTolerance(pos_tol, vel_tol);
    kinematics = drivetrain.getKinematics();
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("***RotateIUntilSeeTags: Init...");
    targetPose = (DriverStation.getAlliance().get() == Alliance.Blue) ? blueTarget : redTarget;
    targetID = (DriverStation.getAlliance().get() == Alliance.Blue) ? blueID : redID;
    
    timer.restart();
    currentPose = drivetrain.getPose();
    targetRot = (Math.atan2(currentPose.getTranslation().getY() - targetPose.getY(),
        currentPose.getTranslation().getX() - targetPose.getX())) // [-pi, pi]
        * 180 / Math.PI ;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    calculate();
    drivetrain.drive(outputModuleState);
  }

  private void calculate() {
    currentPose = drivetrain.getPose();
    outputModuleState = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
        0,
        0,
        pid.calculate(currentPose.getRotation().getDegrees(), targetRot),
        currentPose.getRotation()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("***RotateIUntilSeeTags: End...");
    timer.stop();
  }
    /**
   * Check if limelight can see the target.
   * 
   * @param tagID tagID to check for in limelight
   * @return {@code true} if the target is found in limelight, {@code false} if
   *         not.
   */
  boolean checkForTarget(int tagID) {
    LimelightTarget_Fiducial[] tags = limelight.getAprilTagsFromHelper();
    for (LimelightTarget_Fiducial tag : tags) {
      if ((int)tag.fiducialID == tagID) {
        return true;
      }
    }
    return false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return checkForTarget(targetID) || timer.hasElapsed(1);
  }

}
