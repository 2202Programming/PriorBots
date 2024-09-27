package frc.lib2202.command.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.builder.RobotLimits;
import frc.lib2202.subsystem.hid.HID_Xbox_Subsystem;
import frc.lib2202.subsystem.swerve.SwerveDrivetrain;

/* Current driving behavior:
  Starts in field centric
  B will toggle between field centric and intake centric
  Holding right trigger will switch to hub centric until you let go, then it will go back to original mode
          (either field or intake centric, depending what you started in)
  If in intake centric and you try to rotate with left joystick, will drop back to field centric mode.
*/
public class RobotCentricDrive extends Command {

  final SwerveDrivetrain drivetrain;
  final HID_Xbox_Subsystem dc;
  final SwerveDriveKinematics kinematics;
  final RobotLimits limits;

  // output to Swerve Drivetrain
  double xSpeed, ySpeed, rot;
  SwerveModuleState[] output_states;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(3);
  final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(3);
  final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

  double log_counter = 0;

public RobotCentricDrive() {
    this.dc = RobotContainer.getSubsystem("DC");       //driverControls
    this.drivetrain = RobotContainer.getSubsystem(SwerveDrivetrain.class);
    this.limits = RobotContainer.getRobotSpecs().getRobotLimits();

    addRequirements(drivetrain);
    this.kinematics = drivetrain.getKinematics();
  }

  public RobotCentricDrive(SwerveDrivetrain drivetrain, HID_Xbox_Subsystem dc) {
    this.dc = dc;
    this.drivetrain = drivetrain;
    this.limits = RobotContainer.getRobotSpecs().getRobotLimits();

    addRequirements(drivetrain);
    this.kinematics = drivetrain.getKinematics();
  }


  @Override
  public void initialize() {
    SmartDashboard.putBoolean("RobotCentricDrive", true);
  }

  void calculate() {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    xSpeed = xspeedLimiter.calculate(dc.getVelocityX()) * limits.kMaxSpeed;
    ySpeed = yspeedLimiter.calculate(dc.getVelocityY()) * limits.kMaxSpeed;
    rot = rotLimiter.calculate(dc.getXYRotation()) * limits.kMaxAngularSpeed;

    // Clamp speeds/rot from the Joysticks
    xSpeed = MathUtil.clamp(xSpeed, -limits.kMaxSpeed, limits.kMaxSpeed);
    ySpeed = MathUtil.clamp(ySpeed, -limits.kMaxSpeed, limits.kMaxSpeed);
    rot = MathUtil.clamp(rot, -limits.kMaxAngularSpeed, limits.kMaxAngularSpeed);

    output_states = kinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rot));
  }

  @Override
  public void execute() {
    calculate();
    drivetrain.drive(output_states);
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("RobotCentricDrive", false);
    drivetrain.stop();
  }

}
