// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib2202.subsystem.Swerve;

import com.pathplanner.lib.path.auto.AutoBuilder;
import com.pathplanner.lib.path.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.path.util.PIDConstants;
import com.pathplanner.lib.path.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.base.Constants;
import frc.base.RobotContainerOrig;
import frc.robot2023.subsystems.Sensors.Limelight_Subsystem;
import frc.robot2023.subsystems.Sensors.Sensors_Subsystem;
import frc.robot2023.subsystems.Sensors.Sensors_Subsystem.EncoderID;
import frc.robot2023.subsystems.Swerve.Config.CANConfig;
import frc.robot2023.subsystems.Swerve.Config.ChassisConfig;
import frc.robot2023.subsystems.Swerve.Config.ChassisInversionSpecs;
import frc.robot2023.subsystems.Swerve.Config.WheelOffsets;
import frc.robot.util.ModMath;
import frc.robot.util.VisionWatchdog;

public class SwerveDrivetrain extends SubsystemBase {

  static final double Bearing_Tol = Math.toRadians(0.5); // limit bearing

  // cc is the chassis config for all our pathing math
  private final ChassisConfig cc = RobotContainerOrig.getRobotSpecs().getChassisConfig(); // chassis config
  private final WheelOffsets wc = RobotContainerOrig.getRobotSpecs().getWheelOffset(); // wc = wheel config
  private final ChassisInversionSpecs is = RobotContainerOrig.getRobotSpecs().getChassisInversionSpecs(); //is = invert spec
  private final CANConfig cac = RobotContainerOrig.getRobotSpecs().getCANConfig();
  /**
   *
   * Modules are in the order of - Front Left, Front Right, Back Left, Back Right
   * 
   * Positive x --> represent moving toward the front of the robot
   * Positive y --> represent moving toward the left of the robot
   * [m]
   * https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html#constructing-the-kinematics-object
   */
  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      new Translation2d(cc.XwheelOffset, cc.YwheelOffset), // Front Left
      new Translation2d(cc.XwheelOffset, -cc.YwheelOffset), // Front Right
      new Translation2d(-cc.XwheelOffset, cc.YwheelOffset), // Back Left
      new Translation2d(-cc.XwheelOffset, -cc.YwheelOffset) // Back Right
  );
  final private SwerveDriveOdometry m_odometry;
  private Pose2d m_pose;
  @SuppressWarnings("unused")
  private Pose2d old_pose;
  final private VisionWatchdog watchdog;

  private SwerveModuleState[] meas_states; // measured wheel speed & angle
  private SwerveModulePosition[] meas_pos = new SwerveModulePosition[] {
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition()
  };

  // sensors and our mk3 modules
  private final Sensors_Subsystem sensors;
  private final SwerveModuleMK3[] modules;

  // used to update postion esimates
  double kTimeoffset = .1; // [s] measurement delay from photonvis
  private final Limelight_Subsystem limelight;

  // Network tables 
  public final String NT_Name = "DT"; 
  final private NetworkTable table= NetworkTableInstance.getDefault().getTable(NT_Name);

  // ll pose updating
  private NetworkTableEntry nt_x_diff;
  private NetworkTableEntry nt_y_diff;
  private NetworkTableEntry nt_yaw_diff;    
  private boolean visionPoseUsingRotation = true;
  private boolean visionPoseEnabled = true;

  //private int timer;
  // private double currentBearing = 0;
  private double filteredBearing = 0;
  private double filteredVelocity = 0;

  // Creates a new Single-Pole IIR filter
  // Time constant is 0.1 seconds
  // Period is 0.02 seconds - this is the standard FRC main loop period
  // private LinearFilter bearingFilter = LinearFilter.singlePoleIIR(0.1,
  // Constants.DT);
  // private LinearFilter velocityFilter = LinearFilter.singlePoleIIR(0.1,
  // Constants.DT);

  public final SwerveDrivePoseEstimator m_poseEstimator_ll;
  public final SwerveDrivePoseEstimator m_poseEstimator_pv;
  private double x_diff; // [m]
  private double y_diff; // [m]
  private double yaw_diff; // [deg]

  private Pose2d llPose;
  private Pose2d pvPose;
  public final Field2d m_field = new Field2d();

  public SwerveDrivetrain() {
    sensors = RobotContainerOrig.getSubsystem(Sensors_Subsystem.class);
    limelight = RobotContainerOrig.getSubsystemOrNull(Limelight_Subsystem.class);  //we can deal with no LL
    watchdog = new VisionWatchdog(3.0);

    var MT = CANSparkMax.MotorType.kBrushless;
    modules = new SwerveModuleMK3[] {
        // Front Left
        new SwerveModuleMK3(new CANSparkMax(cac.FL_MODULE.DRIVE_MOTOR_ID, MT), new CANSparkMax(cac.FL_MODULE.ANGLE_MOTOR_ID, MT),
            wc.CC_FL_OFFSET, sensors.getCANCoder(EncoderID.FrontLeft), is.FL.kAngleMotorInvert,
            is.FL.kAngleCmdInvert, is.FL.kDriveMotorInvert, "FL"),
        // Front Right
        new SwerveModuleMK3(new CANSparkMax(cac.FR_MODULE.DRIVE_MOTOR_ID, MT), new CANSparkMax(cac.FR_MODULE.ANGLE_MOTOR_ID, MT),
            wc.CC_FR_OFFSET, sensors.getCANCoder(EncoderID.FrontRight), is.FR.kAngleMotorInvert,
            is.FR.kAngleCmdInvert, is.FR.kDriveMotorInvert, "FR"),
        // Back Left
        new SwerveModuleMK3(new CANSparkMax(cac.BL_MODULE.DRIVE_MOTOR_ID, MT), new CANSparkMax(cac.BL_MODULE.ANGLE_MOTOR_ID, MT),
            wc.CC_BL_OFFSET, sensors.getCANCoder(EncoderID.BackLeft), is.BL.kAngleMotorInvert,
            is.BL.kAngleCmdInvert, is.BL.kDriveMotorInvert, "BL"),
        // Back Right
        new SwerveModuleMK3(new CANSparkMax(cac.BR_MODULE.DRIVE_MOTOR_ID, MT), new CANSparkMax(cac.BR_MODULE.ANGLE_MOTOR_ID, MT),
            wc.CC_BR_OFFSET, sensors.getCANCoder(EncoderID.BackRight), is.BR.kAngleMotorInvert,
            is.BR.kAngleCmdInvert, is.BR.kDriveMotorInvert, "BR") };

    /*
     * Here we use SwerveDrivePoseEstimator so that we can fuse odometry readings.
     * The numbers used below are robot specific, and should be tuned.
     */
    m_poseEstimator_ll = new SwerveDrivePoseEstimator(
        kinematics,
        sensors.getRotation2d(),
        meas_pos,
        new Pose2d(), // initial pose ()
        VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)), // std x,y, heading from odmetry
        VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))); // std x, y heading from vision

    m_poseEstimator_pv = new SwerveDrivePoseEstimator(
        kinematics,
        sensors.getRotation2d(),
        meas_pos,
        new Pose2d(), // initial pose ()
        VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)), // std x,y, heading from odmetry
        VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))); // std x, y heading from vision

    m_odometry = new SwerveDriveOdometry(kinematics, sensors.getRotation2d(), meas_pos);
    meas_states = kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0));
    m_pose = m_odometry.update(sensors.getRotation2d(), meas_pos);

    // ll pose estimating
    nt_x_diff = table.getEntry("vision_x_diff");
    nt_y_diff = table.getEntry("vision_y_diff");
    nt_yaw_diff = table.getEntry("vision_yaw_diff");
    SmartDashboard.putData("Field", m_field);

    offsetDebug();

    // Configure the AutoBuilder last
    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::autoPoseSet, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(7.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(7.0, 0.0, 0.0), // Rotation PID constants
            frc.base.DriveTrain.kMaxSpeed, // Max module speed, in m/s
            0.4, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig()), // Default path replanning config. See the API for the options here
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
    );

  }

  private void offsetDebug() {
    periodic(); // run to initialize module values
    double offsetFL = wc.CC_FL_OFFSET;
    double measuredFL = modules[0].m_internalAngle;

    double offsetFR = wc.CC_FR_OFFSET;
    double measuredFR = modules[1].m_internalAngle;

    double offsetBL = wc.CC_BL_OFFSET;
    double measuredBL = modules[2].m_internalAngle;

    double offsetBR = wc.CC_BR_OFFSET;
    double measuredBR = modules[3].m_internalAngle;

    System.out.println("================Offsets==================");
    System.out.println("FL: offset " + offsetFL + ", measured " + measuredFL + ", should be "
        + ModMath.fmod360_2(offsetFL - measuredFL));
    System.out.println("FR: offset " + offsetFR + ", measured " + measuredFR + ", should be "
        + ModMath.fmod360_2(offsetFR - measuredFR));
    System.out.println("BL: offset " + offsetBL + ", measured " + measuredBL + ", should be "
        + ModMath.fmod360_2(offsetBL - measuredBL));
    System.out.println("BR: offset " + offsetBR + ", measured " + measuredBR + ", should be "
        + ModMath.fmod360_2(offsetBR - measuredBR));
    System.out.println("============Offsets Done==============");
  }

  public void drive(SwerveModuleState[] states) {
    // this.cur_states = states; //keep copy of commanded states so we can stop()
    // withs

    // if any one wheel is above max obtainable speed, reduce them all in the same
    // ratio to maintain control
    // SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveTrain.kMaxSpeed);

    // output the angle and velocity for each module
    for (int i = 0; i < states.length; i++) {
      modules[i].setDesiredState(states[i]);
    }
  }

  // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
  public void driveRobotRelative(ChassisSpeeds chassisSpeed) {
    drive(kinematics.toSwerveModuleStates(chassisSpeed));
  }

  // used for testing
  public void testDrive(double speed, double angle) {
    // output the angle and speed (meters per sec) for each module
    for (int i = 0; i < modules.length; i++) {
      modules[i].setDesiredState(new SwerveModuleState(speed, new Rotation2d(Math.toRadians(angle))));
    }
  }

  public SwerveModulePosition[] getSwerveModulePositions() {
    return meas_pos;
  }

  @Override
  public void periodic() {
    // update data from each of the swerve drive modules.
    for (int i = 0; i < modules.length; i++) {
      modules[i].periodic();
      meas_states[i].speedMetersPerSecond = modules[i].getVelocity();
      meas_states[i].angle = meas_pos[i].angle = modules[i].getAngleRot2d();
      meas_pos[i].distanceMeters = modules[i].getPosition();
    }

    updateOdometry(); 

    /*
     * BEARING STUFFS FROM HERE
     * 
     * // from -PI to +PI (radians)
     * // -dpl - I am not sure this is right way to get a tolerance/filterd
     * // bearing
     * // It is a small change in x/y that needs to be checked for valid atan2()
     * // really should use Vy/Vx.
     * double temp = Math.atan2(m_pose.getY() - old_pose.getY(), m_pose.getX() -  * old_pose.getX());
     * // Changed from !=0 to include tol variable
     * if (Math.abs(temp) < Bearing_Tol) { // remove singularity when moving too
     * slow - otherwise lots of jitter
     * currentBearing = temp;
     * // convert this to degrees in the range -180 to 180
     * currentBearing = Math.toDegrees(currentBearing);
     * }
     * // run bearing through low pass filter
     * filteredBearing = bearingFilter.calculate(currentBearing);
     * 
     * // velocity assuming period is 0.02 seconds - this is the standard FRC main
     * loop
     * // period
     * filteredVelocity = velocityFilter.calculate(PoseMath.poseDistance(m_pose,
     * old_pose) / 0.02);
     * 
     */
    m_field.setRobotPose(m_odometry.getPoseMeters());
  }

  public void simulationInit() {
    // WIP placeholder
    // motor/inertia models

  }

  @Override
  public void simulationPeriodic() {
    // WIP
  }

  public SwerveDriveOdometry getOdometry(){
    return m_odometry;
  }

  public SwerveModuleMK3 getMK3(int modID) {
    if ((modID < 0) || (modID > modules.length - 1))
      return null;
    return modules[modID];
  }

  public void setZeroPose() {
    setPose(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
  }

  // hack to find auto reset point
  public void autoPoseSet(Pose2d pose) {
    System.out.println("***Auto is reseting pose to: " + pose);
    setPose(pose);
  }

  public void setPose(Pose2d pose) {
    m_pose = pose;
    m_odometry.resetPosition(sensors.getRotation2d(), meas_pos, m_pose);

    // keep our vision pose estimators up to date
    if (limelight != null) {
      limelight.setInitialPose(pose, 0.0);
    }
  }

  // reset angle to be zero, but retain X and Y; takes a Rotation2d object
  public void resetAnglePose(Rotation2d rot) {
    m_pose = new Pose2d(getPose().getX(), getPose().getY(), rot);
    m_odometry.resetPosition(sensors.getRotation2d(), meas_pos, m_pose); // updates gryo offset
  }

  public Pose2d getPose() {
    return m_pose;
  }

  public void printPose() {
    System.out
        .println("***POSE X:" + m_pose.getX() + ", Y:" + m_pose.getY() + ", Rot:" + m_pose.getRotation().getDegrees());
  }

  // bearing should be to or from *what*? split out?
  public double getBearing() {
    return filteredBearing;
  }

  public double getVelocity() {
    return filteredVelocity;
  }

  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(meas_states);
  }

  public ChassisSpeeds getFieldRelativeSpeeds() {
    return new ChassisSpeeds(
        getChassisSpeeds().vxMetersPerSecond * sensors.getRotation2d().getCos()
            - getChassisSpeeds().vyMetersPerSecond * sensors.getRotation2d().getSin(),
        getChassisSpeeds().vyMetersPerSecond * sensors.getRotation2d().getCos()
            + getChassisSpeeds().vxMetersPerSecond * sensors.getRotation2d().getSin(),
        getChassisSpeeds().omegaRadiansPerSecond);
  }

  /**
   * stop() - zero the current state's velocity component and leave angles as they
   * are
   */
  public void stop() {
    SwerveModuleState state = new SwerveModuleState();
    state.speedMetersPerSecond = 0.0;
    // output the angle and velocity for each module
    for (int i = 0; i < modules.length; i++) {
      state.angle = Rotation2d.fromDegrees(modules[i].getAngle());
      modules[i].setDesiredState(state);
    }
  }

  public void setBrakeMode() {
    for (int i = 0; i < modules.length; i++) {
      modules[i].setBrakeMode();
    }
    System.out.println("***BRAKES ENGAGED***");
  }

  public void setCoastMode() {
    for (int i = 0; i < modules.length; i++) {
      modules[i].setCoastMode();
    }
    System.out.println("***BRAKES RELEASED***");
  }

  /** Updates the field relative position of the robot. */
  void updateOdometry() {
    // update states
    old_pose = m_pose;
    m_pose = m_odometry.update(sensors.getRotation2d(), meas_pos);

    // vision from here down
    if (limelight != null) {
      m_poseEstimator_ll.update(sensors.getRotation2d(), meas_pos); // this should happen every robot cycle, regardless
                                                                    // of vision targets.
      llPoseEstimatorUpdate();
    }

    //TODO: Currently, the limelight is wrong. Whenever you move just past the stage, it appears
    //that something is off. Specifically, the robot dramatically shifts its position. Additionally,
    // this often causes the autonomous program to be off. Because of this, we're missing out on
    // not only crucial points needed to win matches, but also the potential melody ranking point.
    // Thus, we must ADD UNIT AND CONSTANT FIX TO AVOID BAD UPDATE FROM LIMELIGHT>>>>
    if ((limelight != null) && (llPose != null) && (limelight.getNumApriltags() > 0) && 
    (limelight.getTA() > 0.13) && (Math.abs(modules[0].getVelocity()) < 2.5)) {
      Pose2d prev_m_Pose = m_pose;
      if (visionPoseEnabled) {
        watchdog.update(prev_m_Pose, llPose);
        if (visionPoseUsingRotation) {
          setPose(llPose); // update robot pose from swervedriveposeestimator, include vision-based
                           // rotation
        } else {
          // update robot translation from swervedriveposeestimator, do not update rotation
          setPose(new Pose2d(llPose.getTranslation(), prev_m_Pose.getRotation())); 
        }
      }
      x_diff = Math.abs(prev_m_Pose.getX() - m_pose.getX());
      y_diff = Math.abs(prev_m_Pose.getY() - m_pose.getY());
      yaw_diff = Math.abs(prev_m_Pose.getRotation().getDegrees() - m_pose.getRotation().getDegrees());
      
      // vision pose updating NTs
      nt_x_diff.setDouble(x_diff);
      nt_y_diff.setDouble(y_diff);
      nt_yaw_diff.setDouble(yaw_diff);
    }

  }

  void llPoseEstimatorUpdate() {

    if (limelight.getNumApriltags() > 0) {
      // this should happen only if we have a tag in view
      // OK if it is run only intermittanly. Uses latency of vision pose.
      m_poseEstimator_ll.addVisionMeasurement(limelight.getBluePose(), limelight.getVisionTimestamp());

      llPose = m_poseEstimator_ll.getEstimatedPosition();
    }
  }

  public Pose2d getLLEstimate() {
    return llPose;
    //return (llPose != null) ? new Pose2d(llPose.getTranslation(), llPose.getRotation()) : null;
  }

  public Pose2d getPVEstimate() {
    return pvPose;
    //return (pvPose != null) ? new Pose2d(pvPose.getTranslation(), pvPose.getRotation()) : null;
  }

  public double getDistanceToTranslation(Translation2d targetTranslation) {
    return Math.sqrt(
           Math.pow(getPose().getTranslation().getX() - targetTranslation.getX(), 2)
              + Math.pow(getPose().getTranslation().getY() - targetTranslation.getY(), 2));

  }

  public void disableVisionPoseRotation() {
    visionPoseUsingRotation = false;
    System.out.println("*** Vision pose updating rotation disabled***");
  }

  public void enableVisionPoseRotation() {
    visionPoseUsingRotation = true;
    System.out.println("*** Vision pose updating rotation enabled***");
  }

  public void enableVisionPose() {
    visionPoseEnabled = true;
    System.out.println("*** Vision updating pose enabled***");
  }

  public void disableVisionPose() {
    visionPoseEnabled = false;
    System.out.println("*** Vision updating pose disabled***");
  }

}