// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2024.commands.Shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib2202.builder.RobotContainer;
import frc.robot2024.Constants.Tag_Pose;
import frc.robot2024.subsystems.ShooterServo;
import frc.lib2202.subsystem.OdometryInterface;

public class SpeakerShooter extends InstantCommand {
  //SwerveDrivetrain drivetrain;
  OdometryInterface odo;
  
  // Keep these constants here, not needed elsewhere
  // TODO:Check this value. - Estimate
  private final double SHOOTER_Y_OFFSET = 0.55; // [m] pivotal point of shooter from the center
  private final double SHOOTER_Z_OFFSET = 0.17; // [m] shooter z position from floor
  private final double SPEAKER_HEIGHT = 2.1; // [m] speaker height from the floor

  private final double angle_adjustment = 5.25; // [deg] angle gain/lose for tuning

  // RobotPose with polar coordinate(origin is tag)
  private double radius; // [m] distance from the speaker to the robot
  // [deg] 0 is the condition where the robot is in front of the speaker
  // +- 90 degree from that zero. but uses absolute value becuase of simiticity of
  // shooting condition.
  // So range of this is 0-90 which simlply means it is just how far off from the
  // front of the speaker
  private double rot;

  // shooter angle [deg]
  private double shooter_angle;
  private double dX;
  private double dY;
  private boolean rpm_given = false;
  private double rpm;

  /**
   * Command shooting speaker
   * <p>
   * Requires: ShooterServo and limelight detecting specified tag.
   * Calculate the angle and RPM to shoot the speaker
   * </p>
   * <p>
   * Shooter angle- will be calculated by the distance and geometry.
   * </p>
   * RPM- will be calculated using formula based of polar coordinate determined
   * the target is the vertex of the polar coordinate
   */
  public SpeakerShooter() {
    //drivetrain = RobotContainer.getSubsystem(SwerveDrivetrain.class);
    odo = RobotContainer.getSubsystemOrNull("odometry");
    rpm_given = false;
    // addRequirements(shooter) not be needed, this cmd schedules another
  }

  /**
   * Using a given RPM purpose setting rpm by parameter
   * 
   * @param rpm shooter rpm
   */
  public SpeakerShooter(double rpm) {
    this();
    rpm_given = true;
    this.rpm = rpm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // get robot pose in field coordinates
    Pose2d pose = odo.getPose();
    Translation2d pos = pose.getTranslation();

    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      dX = Math.abs(pos.getX() - Tag_Pose.ID7.location.getX());
      dY = Math.abs(pos.getY() - Tag_Pose.ID7.location.getY());
    } else {
      dX = Math.abs(pos.getX() - Tag_Pose.ID4.location.getX());
      dY = Math.abs(pos.getY() - Tag_Pose.ID4.location.getY());
    }
    radius = Math.sqrt(Math.pow(dX, 2) + Math.pow(dY, 2));
    rot = Math.abs(Math.atan2(dY, dX)) * 180 / Math.PI;
    System.out.println("SpeakerShooter---Shooting Position: [" + radius + "," + rot + "]");

    // get shooter angle
    shooter_angle = Math.atan2((SPEAKER_HEIGHT - SHOOTER_Z_OFFSET), (radius - SHOOTER_Y_OFFSET)) * 180 / Math.PI
        + angle_adjustment;
    System.out.println("SpeakerShooter---Before:" + shooter_angle);
    shooter_angle = MathUtil.clamp(shooter_angle, ShooterServo.MIN_DEGREES, ShooterServo.MAX_DEGREES);
    // get RPM @see src/main/python/regression.py
    // if rpm_given mode, use the rpm from parameter
    if (!rpm_given) {
      rpm = 7205.19 + -3266.57 * dX + -3266.57 * dY + 935.96 * Math.pow(dX, 2) + 731.54 * dX * dY
          + 935.96 * Math.pow(dY, 2);
      rpm = MathUtil.clamp(rpm, 2000, 3500);
    }

    Command shooterCmd = new ShooterServoSequence(shooter_angle, rpm);
    shooterCmd.schedule();

    System.out.println("SpeakerShooter--- SSS scheduled @ Angle: " +
        shooter_angle + " [deg]" +
        " RPM: " + rpm + " [rpm]");

  }
}