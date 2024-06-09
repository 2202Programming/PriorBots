package frc.robot2024.commands.Swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot2023.subsystems.Sensors.LimelightHelpers.LimelightTarget_Fiducial;
import frc.base.Constants;
import frc.base.RobotContainerOrig;
import frc.base.Constants.DriveTrain;
import frc.base.Constants.Tag_Pose;
import frc.robot2023.subsystems.Sensors.Limelight_Subsystem;
import frc.robot2023.subsystems.Swerve.SwerveDrivetrain;
import frc.robot2023.subsystems.hid.HID_Xbox_Subsystem;

/*
  Driver controls the robot using field coordinates.
    X,Y, Rotation
  
  While the robot has a note, the robot will _always_ be pointing at the target
  This command is envisioned to be used in a WhileTrue() similar to RobotCentric
*/
public class TargetCentricDrive extends Command {

  public enum state {
    Init("Init"),
    BlindTrack("BlindTrack"),
    TagTrack("TagTrack");

    private String name;

    private state(String name) {
      this.name = name;
    }

    public String toString() {
      return name;
    }
  }

  private state currentState;
  final SwerveDrivetrain drivetrain;
  final SwerveDriveKinematics kinematics;
  final HID_Xbox_Subsystem dc;
  private final Limelight_Subsystem limelight;
  
  // Limelight PID
  private PIDController blindPid;
  private final double blindPid_kp = 3.0;
  private final double blindPid_ki = 0.0;
  private final double blindPid_kd = 0.0;
  private double targetRot;
  private Pose2d currentPose;
  private Translation2d targetPose; // Position want to face to

  // odometery PID
  PIDController centeringPid;
  double centering_kP = 2.0; //used to be 3.5 when we were in degrees
  double centering_kI = 0;
  double centering_kD = 0;
  double centeringPidOutput = 2.0;
  double vel_tol = 1.0 / 57.3; // [rad/sec]
  double pos_tol_blind = 5.0/ 57.3; // [rad]
  double pos_tol_tag = 2.5;
  double max_rot_rate = 45.0; // [deg/s]
  double min_rot_rate = 6.0;
  private double TagID;

  // output to Swerve Drivetrain
  double xSpeed, ySpeed, rot;
  Rotation2d currrentHeading;
  SwerveModuleState[] output_states;
  boolean hasTarget = false;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(3);
  final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(3);
  final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

  public TargetCentricDrive() {
    this.dc = RobotContainerOrig.getSubsystem("DC"); // driverControls
    this.drivetrain = RobotContainerOrig.getSubsystem(SwerveDrivetrain.class);
    this.kinematics = drivetrain.getKinematics();
    this.limelight = RobotContainerOrig.getSubsystem(Limelight_Subsystem.class);

    addRequirements(drivetrain); // This means we area read-only for everything but drivetrain
                                 // limelight and intake are used as read-only

    // PID for when tag is in view
    centeringPid = new PIDController(centering_kP, centering_kI, centering_kD);
    centeringPid.setTolerance(pos_tol_tag, vel_tol);

    // PID for when tag is not visable
    blindPid = new PIDController(blindPid_kp, blindPid_ki, blindPid_kd); //[rad]
    blindPid.enableContinuousInput(-Math.PI, Math.PI); //[rad]
    blindPid.setTolerance(pos_tol_blind, vel_tol);  // Not being used

  }

  @Override
  public void initialize() {
    TagID = (DriverStation.getAlliance().get() == Alliance.Blue) ? 7: 4;
    targetPose = Tag_Pose.tagLocations[(int) TagID];
    currentState = state.Init;
    SmartDashboard.putString("TargetCentricDrive State", currentState.toString());
  }

  @Override
  public void execute() {
    double tagXfromCenter = checkForTarget(TagID); // checkForTarget is updating tagXfromCenter, hasTarget

    if (hasTarget) {
        currentState = state.TagTrack;
      } else {
        currentState = state.BlindTrack;
      }

    SmartDashboard.putString("TargetCentricDrive State", currentState.toString());

    calculateRotFromOdometery(); // always feed PID, even if rot gets overwritten later.

    switch (currentState) {
      case TagTrack:
        // 3/23/24 Seems to be working  
        calculateRotFromTarget(tagXfromCenter); // has note, can see target tag, close loop via limelight
        break;
      
        case Init: //should never get here
        System.out.println("***Impossible state reached in TargetCentricDrive***");
        break;
      
        case BlindTrack:
        // has note, but can't see target, use odometery for rot (already run)
        break;
    }

    SmartDashboard.putNumber("TargetCentricDrive rot", rot);
    calculate(); // used to calculate X and Y from joysticks, and rotation from one of methods
                 // above
    drivetrain.drive(output_states);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  private void calculateRotFromOdometery() {
    currentPose = drivetrain.getPose();
    targetRot = (Math.atan2(currentPose.getTranslation().getY() - targetPose.getY(),
        currentPose.getTranslation().getX() - targetPose.getX())); // [-pi, pi]
    //targetRot = targetRot - Math.PI; //invert facing to have shooter face target - Not needed for betabot
    SmartDashboard.putNumber("TargetCentricDrive Odo target", targetRot);
    rot = blindPid.calculate(currentPose.getRotation().getRadians(), targetRot); //in radians
  }

  private void calculateRotFromTarget( double tagXfromCenter) {
    centeringPidOutput = centeringPid.calculate(tagXfromCenter, 0.0);
    double min_rot = Math.signum(centeringPidOutput) * min_rot_rate;
    rot = MathUtil.clamp(centeringPidOutput + min_rot, -max_rot_rate, max_rot_rate) / Constants.DEGperRAD; // convert to
                                                                                                           // radians

  }

  void calculateRotFromJoystick() {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.

    rot = rotLimiter.calculate(dc.getXYRotation()) * DriveTrain.kMaxAngularSpeed;

    // Clamp speeds/rot from the Joysticks
    rot = MathUtil.clamp(rot, -DriveTrain.kMaxAngularSpeed, DriveTrain.kMaxAngularSpeed);

  }

  void calculate() { // lets use arguments and returns please
    // X and Y from joysticks; rot from previous calculations
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    xSpeed = xspeedLimiter.calculate(dc.getVelocityX()) * DriveTrain.kMaxSpeed;
    ySpeed = yspeedLimiter.calculate(dc.getVelocityY()) * DriveTrain.kMaxSpeed;

    // Clamp speeds from the Joysticks
    xSpeed = MathUtil.clamp(xSpeed, -DriveTrain.kMaxSpeed, DriveTrain.kMaxSpeed);
    ySpeed = MathUtil.clamp(ySpeed, -DriveTrain.kMaxSpeed, DriveTrain.kMaxSpeed);

    currrentHeading = drivetrain.getPose().getRotation();
    // convert field centric speeds to robot centric
    ChassisSpeeds tempChassisSpeed = (DriverStation.getAlliance().get().equals(Alliance.Blue))
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, currrentHeading)
        : ChassisSpeeds.fromFieldRelativeSpeeds(-xSpeed, -ySpeed, rot, currrentHeading); // if on red alliance you're
                                                                                         // looking at robot from
                                                                                         // opposite. Pose is in blue
                                                                                         // coordinates so flip if red
    output_states = kinematics.toSwerveModuleStates(tempChassisSpeed);
  }

  private double checkForTarget(double tagID) {
    LimelightTarget_Fiducial[] tags = limelight.getAprilTagsFromHelper();
    double tagXfromCenter = 0.0;
    hasTarget = false;
    for (LimelightTarget_Fiducial tag : tags) {
      if (tag.fiducialID == tagID) {
        tagXfromCenter = tag.tx; // update global variables
        hasTarget = true; // update global variables
      }
    }
    SmartDashboard.putNumber("TargetCentricDrive TagX", tagXfromCenter);
    SmartDashboard.putBoolean("TargetCentricDrive hasTarget", hasTarget);
    return tagXfromCenter;
  }

}