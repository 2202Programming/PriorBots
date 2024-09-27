// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.chadbot.commands.Shoot;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.chadbot.Constants.Shooter;
import frc.chadbot.RobotContainer;
import frc.chadbot.subsystems.Limelight_Subsystem;
import frc.chadbot.subsystems.SwerveDrivetrain;

/* Current driving behavior:
  Starts in field centric
  B will toggle between field centric and intake centric
  Holding right trigger will switch to hub centric until you let go, then it will go back to original mode
          (either field or intake centric, depending what you started in)
  If in intake centric and you try to rotate with left joystick, will drop back to field centric mode.
*/


public class LimelightAim extends Command {

  final SwerveDrivetrain drivetrain;
  final SwerveDriveKinematics kinematics;
  final Limelight_Subsystem limelight;

  // output to Swerve Drivetrain
  double xSpeed, ySpeed, rot;
  SwerveModuleState[] output_states;

  // PID for limelight-based heading to a target
  PIDController limelightPid;
  double limelight_kP = Shooter.limelight_default_p;
  double limelight_kI = Shooter.limelight_default_i;
  double limelight_kD = Shooter.limelight_default_d;
  double limelightPidOutput = 0.0;
  
  double r_limelight_kP = limelight_kP;
  double r_limelight_kI = limelight_kI;
  double r_limelight_kD = limelight_kD;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(3);
  final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(3);
  final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);
  final SlewRateLimiter llLimiter = new SlewRateLimiter(3);

  NetworkTable table;
  private NetworkTableEntry NTangleError;
  public final String NT_Name = "Shooter"; 

  double log_counter = 0;
  Rotation2d currentAngle;
  Rotation2d angleError;
  Rotation2d targetAngle;
  Rotation2d velocityCorrectionAngle;

  double min_rot_rate = 6.0;        //about 7.5 deg is min we measured
  double r_min_rot_rate = min_rot_rate;

  final double vel_tol = 10.0;
  final double pos_tol = 2.0;

  //A simplified version of HubCentricDrive, designed for stand-alone limelight aiming during auto
  //will finish when PID error is small enough, to allow auto to progress to next command.

  public LimelightAim() {
    this.drivetrain = RobotContainer.RC().drivetrain;
    addRequirements(drivetrain);
    this.kinematics = drivetrain.getKinematics();
    this.limelight = RobotContainer.RC().limelight;

    // anglePid = new PIDController(angle_kp, angle_ki, angle_kd);
    limelightPid = new PIDController(limelight_kP, limelight_kI, limelight_kD);
    limelightPid.setTolerance(pos_tol, vel_tol);
    table = NetworkTableInstance.getDefault().getTable(NT_Name);
    NTangleError = table.getEntry("/LimelightAim/angleError");

    calculate();

  }

  @Override
  public void initialize() {
    limelight.enableLED(); 
    updateNT();
  }

  void calculate() {
    final double max_rot_rate = 60.0;  //[deg/s]
    double llx = limelight.getFilteredX();  //[deg error]

    // limelight is on the shooter side, so we don't need to worry about flipping target angles
    limelightPid.setSetpoint(0);
   
    limelightPidOutput = limelightPid.calculate(llx);
    angleError = Rotation2d.fromDegrees(limelight.getX()); //approximation of degrees off center

    // Clamp rotation rate to +/- X degrees/sec
    double min_rot = (Math.abs(llx) > pos_tol)  ? - Math.signum(llx) * min_rot_rate : 0.0;
    rot = MathUtil.clamp(limelightPidOutput + min_rot, -max_rot_rate, max_rot_rate) / 57.3;   //clamp in [deg/s] convert to [rad/s]

    currentAngle = drivetrain.getPose().getRotation();
    output_states = kinematics
        .toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, rot, currentAngle));

  }

  @Override
  public void execute() {

    calculate();
    drivetrain.drive(output_states);
    updateNT();
  }

  @Override
  public boolean isFinished(){
    return isReady();
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  void updateNT() {
    log_counter++;
    if ((log_counter%20)==0) {
      // update network tables
      NTangleError.setDouble(angleError.getDegrees());
    }
  }

  public boolean isReady() {
    return limelightPid.atSetpoint();
  }

}
