package frc.lib2202.command.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.builder.RobotLimits;
import frc.lib2202.subsystem.swerve.SwerveDrivetrain;
import frc.lib2202.subsystem.hid.HID_Xbox_Subsystem;

/*
  Driver controls the robot using field coordinates.
    X,Y, Rotation
*/
public class FieldCentricDrive extends Command {

  protected final SwerveDrivetrain drivetrain;
  protected final SwerveDriveKinematics kinematics;
  protected final HID_Xbox_Subsystem dc;
  protected final RobotLimits limits;

  // output to Swerve Drivetrain
  protected double xSpeed, ySpeed, rot;
  protected Rotation2d currrentHeading;
  protected SwerveModuleState[] output_states;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(3);
  final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(3);
  final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);
  
  public FieldCentricDrive() {
    this.dc = RobotContainer.getSubsystem("DC");       //driverControls
    this.drivetrain = RobotContainer.getSubsystem(SwerveDrivetrain.class);
    this.limits = RobotContainer.getRobotSpecs().getRobotLimits();

    addRequirements(drivetrain);
    this.kinematics = drivetrain.getKinematics();
  }

  @Override
  public void initialize() {
     SmartDashboard.putBoolean("FieldCentricDrive Enabled", true);
  }

  protected void calculate() {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    xSpeed = xspeedLimiter.calculate(dc.getVelocityX()) * limits.kMaxSpeed;
    ySpeed = yspeedLimiter.calculate(dc.getVelocityY()) * limits.kMaxSpeed;
    rot = rotLimiter.calculate(dc.getXYRotation()) * limits.kMaxAngularSpeed;

    // Clamp speeds/rot from the Joysticks
    xSpeed = MathUtil.clamp(xSpeed, -limits.kMaxSpeed, limits.kMaxSpeed);
    ySpeed = MathUtil.clamp(ySpeed, -limits.kMaxSpeed, limits.kMaxSpeed);
    rot = MathUtil.clamp(rot, -limits.kMaxAngularSpeed, limits.kMaxAngularSpeed);

    currrentHeading = drivetrain.getPose().getRotation();
    //convert field centric speeds to robot centric
    ChassisSpeeds tempChassisSpeed = (DriverStation.getAlliance().get().equals(Alliance.Blue)) 
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, currrentHeading) 
        : ChassisSpeeds.fromFieldRelativeSpeeds(-xSpeed, -ySpeed, rot, currrentHeading); // if on red alliance you're looking at robot from opposite. Pose is in blue coordinates so flip if red

    output_states = kinematics.toSwerveModuleStates(tempChassisSpeed);
  }

  @Override
  public void execute() {
    calculate();
    drivetrain.drive(output_states);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

}