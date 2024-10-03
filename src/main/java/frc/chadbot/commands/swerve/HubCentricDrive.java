package frc.chadbot.commands.swerve;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.chadbot.Constants;
import frc.chadbot.Constants.DriveTrain;
import frc.chadbot.Constants.Shooter;
import frc.chadbot.subsystems.Limelight_Subsystem;
import frc.lib2202.subsystem.swerve.SwerveDrivetrain;
import frc.chadbot.subsystems.ifx.DriverControls;

/* Current driving behavior:
  Starts in field centric
  B will toggle between field centric and intake centric
  Holding right trigger will switch to hub centric until you let go, then it will go back to original mode
          (either field or intake centric, depending what you started in)
  If in intake centric and you try to rotate with left joystick, will drop back to field centric mode.
*/


public class HubCentricDrive extends DriveCmdClass {

  final SwerveDrivetrain drivetrain;
  final DriverControls dc;
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
  double limelightTarget;

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

  double max_rot_rate = 180.0;  //[deg/s]
  double min_rot_rate = 7;    //6    //about 7.5 deg is min we measured
  double r_min_rot_rate = min_rot_rate;
  double r_max_rot_rate = max_rot_rate;

  final double vel_tol = 30.0;
  final double pos_tol = 3.0;

  final boolean PID_TUNING = false;

  public HubCentricDrive(SwerveDrivetrain drivetrain, DriverControls dc, Limelight_Subsystem limelight) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
    this.dc = dc;
    this.kinematics = drivetrain.getKinematics();
    this.limelight = limelight;

    // anglePid = new PIDController(angle_kp, angle_ki, angle_kd);
    limelightPid = new PIDController(limelight_kP, limelight_kI, limelight_kD);
    limelightPid.setTolerance(pos_tol, vel_tol);
    table = NetworkTableInstance.getDefault().getTable(NT_Name);
    NTangleError = table.getEntry("/HubCentric/angleError");

    calculate();

  }

  @Override
  public void initialize() {
    limelightTarget = 0.0;
    updateNT();
    if (PID_TUNING) firstPIDPrint();
  }

  void calculate() {
    double llx = limelight.getFilteredX();  //[deg error]
    //double llx = limelight.getX();
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    xSpeed = xspeedLimiter.calculate(dc.getVelocityX()) * DriveTrain.kMaxSpeed;
    ySpeed = yspeedLimiter.calculate(dc.getVelocityY()) * DriveTrain.kMaxSpeed;

    // Clamp speeds/rot from the Joysticks
    xSpeed = MathUtil.clamp(xSpeed, -Constants.DriveTrain.kMaxSpeed, Constants.DriveTrain.kMaxSpeed);
    ySpeed = MathUtil.clamp(ySpeed, -Constants.DriveTrain.kMaxSpeed, Constants.DriveTrain.kMaxSpeed);

    // limelight is on the shooter side, so we don't need to worry about flipping target angles
    limelightPid.setSetpoint(limelightTarget);
  
    limelightPidOutput = limelightPid.calculate(llx);
    angleError = Rotation2d.fromDegrees(limelight.getX()); //approximation of degrees off center

    // Clamp rotation rate to +/- X degrees/sec
    // adjust min_rot_rate to barely turn robot when P/I/D are all zero (similar to FF)
    double min_rot = (Math.abs(llx) > pos_tol)  ? - Math.signum(llx) * min_rot_rate : 0.0;
    rot = MathUtil.clamp(limelightPidOutput + min_rot, -max_rot_rate, max_rot_rate) / 57.3;   //clamp in [deg/s] convert to [rad/s]

    currentAngle = drivetrain.getPose().getRotation();
    //convert field centric speeds to robot centric
    ChassisSpeeds tempChassisSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, currentAngle);

    //tip correction is in robot centric
    tempChassisSpeed.vxMetersPerSecond += pitch_correction;
    tempChassisSpeed.vyMetersPerSecond += roll_correction;

    output_states = kinematics
        .toSwerveModuleStates(tempChassisSpeed);

  }

  @Override
  public void execute() {
    if (PID_TUNING) {
      pidPrint();  //these are for PID tuning only
      pidSet();
    }
    calculate();
    drivetrain.drive(output_states);
    updateNT();
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  void updateNT() {
    // SmartDashboard.putBoolean("Limelight Solution Ready", isReady());
    // SmartDashboard.putNumber("Limelight PID Position Error", limelightPid.getPositionError());
    // SmartDashboard.putNumber("Limelight PID Velocity Error", limelightPid.getVelocityError());
    log_counter++;
    if ((log_counter%20)==0) {
      // update network tables
      NTangleError.setDouble(angleError.getDegrees());

    }
  }

  public Rotation2d getAngleError() {
    return this.angleError;
  }

  public boolean isReady() {
    return limelightPid.atSetpoint();
  }

  public void setLimelightTarget(double target) {
    limelightTarget = target;
  }

  public double getLimelightTarget(){
    return limelightTarget;
  }


  private void firstPIDPrint(){
    SmartDashboard.putNumber("Current Limelight P", limelightPid.getP());
    SmartDashboard.putNumber("Current Limelight I", limelightPid.getI());
    SmartDashboard.putNumber("Current Limelight D", limelightPid.getD());
    SmartDashboard.putNumber("Current min rotation rate", min_rot_rate);
    SmartDashboard.putNumber("Current Max rot rate", max_rot_rate);

    SmartDashboard.putNumber("Requested Limelight P", r_limelight_kP);
    SmartDashboard.putNumber("Requested Limelight I", r_limelight_kI);
    SmartDashboard.putNumber("Requested Limelight D", r_limelight_kD);
    SmartDashboard.putNumber("Requested min rotation rate", r_min_rot_rate);
    SmartDashboard.putNumber("Requested Max rot rate", r_max_rot_rate);
  }

  private void pidPrint(){
    SmartDashboard.putNumber("Current Limelight P", limelightPid.getP());
    SmartDashboard.putNumber("Current Limelight I", limelightPid.getI());
    SmartDashboard.putNumber("Current Limelight D", limelightPid.getD());
    SmartDashboard.putNumber("Current min rotation rate", min_rot_rate);
    SmartDashboard.putNumber("Current Max rot rate", max_rot_rate);
  }

  private void pidSet(){
    r_limelight_kP = SmartDashboard.getNumber("Requested Limelight P", r_limelight_kP);
    r_limelight_kI = SmartDashboard.getNumber("Requested Limelight I", r_limelight_kI);
    r_limelight_kD = SmartDashboard.getNumber("Requested Limelight D", r_limelight_kD);
    r_min_rot_rate = SmartDashboard.getNumber("Requested min rotation rate", r_min_rot_rate);
    r_max_rot_rate = SmartDashboard.getNumber("Requested Max rot rate", r_max_rot_rate);

    if((r_limelight_kP!=limelight_kP) || (r_limelight_kI!=limelight_kI) || (r_limelight_kD != limelight_kD)){
      limelight_kP=r_limelight_kP;
      limelight_kI=r_limelight_kI;
      limelight_kD=r_limelight_kD;
      limelightPid.setPID(limelight_kP, limelight_kI, limelight_kD);
      limelightPid.reset();
      System.out.println("***PID UPDATED***");
    }
    max_rot_rate = r_max_rot_rate;
    min_rot_rate = r_min_rot_rate;

  }

}
