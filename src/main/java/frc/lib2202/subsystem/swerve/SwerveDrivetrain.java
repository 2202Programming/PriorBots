// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.lib2202.subsystem.swerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
//wip import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
//import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.subsystem.swerve.config.ChassisConfig;
import frc.lib2202.subsystem.swerve.config.ModuleConfig;
import frc.lib2202.util.ModMath;

public class SwerveDrivetrain extends SubsystemBase {
  static final String canBusName = "rio";
  static final double longWaitSeconds = 1.0; // cancode config wait

  // cc is the chassis config for all our pathing math
  // final RobotLimits limits
  final ChassisConfig cc; // from robotSpecs
  final ModuleConfig mc[]; // from robotSpecs

  /**
   *
   * Modules are in the order of - Front Left, Front Right, Back Left, Back Right
   * 
   * Positive x --> represent moving toward the front of the robot [m]
   * Positive y --> represent moving toward the left of the robot [m]
   *
   * https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html#constructing-the-kinematics-object
   */
  final SwerveDriveKinematics kinematics;
  final SwerveDriveOdometry m_odometry;
  Pose2d m_pose; // pose based strictly on the odometry

  // controls behavior of visionPposeestimator
  boolean visionPoseUsingRotation = true;
  boolean visionPoseEnabled = true;

  // Swerver States and positions
  SwerveModuleState[] meas_states; // measured wheel speed & angle
  SwerveModulePosition[] meas_pos; // distance & angle for each module

  // sensors and our mk3 modules
  final IHeadingProvider sensors;
  final SwerveModuleMK3[] modules;
  final CANcoder canCoders[];

  // pose/field measurements
  public final Field2d m_field; // Field2d based on odometry only
  private RobotConfig GUIconfig;


  public SwerveDrivetrain() {
    m_field = new Field2d();

    cc = RobotContainer.getRobotSpecs().getChassisConfig();
    mc = RobotContainer.getRobotSpecs().getModuleConfigs();

    // Coords checked: Left +Y offset, right -Y offset, +X, front -x back.
    // See:
    // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
    // match order to ModuleConfig[] in RobotSpec_<robot name>.java
    kinematics = new SwerveDriveKinematics(
        new Translation2d(cc.XwheelOffset, cc.YwheelOffset), // Front Left
        new Translation2d(cc.XwheelOffset, -cc.YwheelOffset), // Front Right
        new Translation2d(-cc.XwheelOffset, cc.YwheelOffset), // Back Left
        new Translation2d(-cc.XwheelOffset, -cc.YwheelOffset) // Back Right
    );

    // allocate space for measured positions, initialized to zeros
    meas_pos = new SwerveModulePosition[] {
        new SwerveModulePosition(), new SwerveModulePosition(),
        new SwerveModulePosition(), new SwerveModulePosition()
    };

    sensors = RobotContainer.getRobotSpecs().getHeadingProvider();
    canCoders = new CANcoder[mc.length];
    modules = new SwerveModuleMK3[mc.length];

    // create cancoders and swerve modules
    for (int i = 0; i < mc.length; i++) {
      canCoders[i] = initCANcoder(mc[i].CANCODER_ID, mc[i].kAngleOffset);
      modules[i] = new SwerveModuleMK3(
          new SparkMax(mc[i].DRIVE_MOTOR_ID, SparkMax.MotorType.kBrushless),
          new SparkMax(mc[i].ANGLE_MOTOR_ID, SparkMax.MotorType.kBrushless),
          canCoders[i],
          mc[i].kAngleMotorInvert,
          mc[i].kAngleCmdInvert,
          mc[i].kDriveMotorInvert,
          mc[i].id.toString());

      /* Speed up signals to an appropriate rate */
      // long term wip BaseStatusSignal.setUpdateFrequencyForAll(100,
      // canCoders[i].getPosition(), canCoders[i].getVelocity());
    }

    m_odometry = new SwerveDriveOdometry(kinematics, sensors.getRotation2d(), meas_pos);
    meas_states = kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0));
    m_pose = m_odometry.update(sensors.getRotation2d(), meas_pos);

    configureAutoBuilder();
    offsetDebug();
  }

  /**
   * initCANcoder() - setup cancoder the way we need them.
   * This CANcoder returns value in rotation with phoenix 6 [-0.5, 0.5)
   * rotations (clock wise is positive).
   * canBusName set to "rio" at top of module
   * 
   * @param cc_ID
   * @param cc_offset_deg [+/- 180 deg]
   * 
   * @return CANcoder just initialized
   */
  private CANcoder initCANcoder(int cc_ID, double cc_offset_deg) {
    CANcoder canCoder = new CANcoder(cc_ID, canBusName);
    StatusSignal<Angle> abspos = canCoder.getAbsolutePosition().waitForUpdate(longWaitSeconds, true);
    StatusSignal<Angle> pos = canCoder.getPosition().waitForUpdate(longWaitSeconds, true);
    /*
     * System.out.println("CC(" + cc_ID + ") before: " +
     * "\tabspos = " + abspos.getValue() + " (" + abspos.getValue()*360.0+" deg)" +
     * "\tpos = " + pos.getValue() + " (" + pos.getValue()*360.0 +" deg)" );
     */
    CANcoderConfiguration configs = new CANcoderConfiguration();
    configs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5; //AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    configs.MagnetSensor.MagnetOffset = cc_offset_deg / 360.0; // put offset deg on +/- 0.5 range
    configs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    canCoder.clearStickyFaults(longWaitSeconds);

    // update mag offset and check status, report errors
    StatusCode status = canCoder.getConfigurator().apply(configs, longWaitSeconds);
    if (!status.isOK()) {
      System.out.println("Warning CANCoder(" + cc_ID + ") returned " +
          status.toString() + " on applying confg. Retrying");
      SwerveModuleMK3.sleep(100);
      canCoder.clearStickyFaults(longWaitSeconds);
      status = canCoder.getConfigurator().apply(configs, longWaitSeconds);
      System.out.println("CANCoder(" + cc_ID + ") status on retry: " + status.toString() + " moving on, good luck.");
    }
    canCoder.clearStickyFaults(longWaitSeconds);

    // Re-read sensor, blocking calls
    abspos.waitForUpdate(longWaitSeconds, true);
    pos.waitForUpdate(longWaitSeconds, true);
    /*
     * System.out.println("CC(" + cc_ID + ")  after: "+
     * "\tabspos = " + abspos.getValue() + " (" + abspos.getValue()*360.0+" deg)" +
     * "\tpos = " + pos.getValue() + " (" + pos.getValue()*360.0 + " deg)" );
     */
    return canCoder;
  }

  // debugging print for mag-offsets of canCoders
  private void offsetDebug() {
    periodic(); // run to initialize module values
    System.out.println("================OffsetDebug==================");
    for (int i = 0; i < mc.length; i++) {
      double offset = mc[i].kAngleOffset;
      double measured = modules[i].m_internalAngle;
      double cc_measured = modules[i].m_externalAngle;
      System.out.println(mc[i].id.toString() + ": offset=" + offset + ", internal=" + measured +
          " cc_meas=" + cc_measured + ", if zero-aligned, set mag offset = " +
          ModMath.fmod360_2(offset - cc_measured));
    }
    System.out.println("============OffsetDebug Done==============");
  }

  public void drive(SwerveModuleState[] states) {
    // this.cur_states = states; //keep copy of commanded states so we can stop()
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

    // this pose is only based on odometry, no vision
    m_pose = m_odometry.update(sensors.getRotation2d(), meas_pos);
    m_field.setRobotPose(m_odometry.getPoseMeters());
  }

  // AutoBuilder for PathPlanner - uses internal static vars in AutoBuilder
  void configureAutoBuilder() {
    try{
      GUIconfig = RobotConfig.fromGUISettings();

    // Configure the AutoBuilder last
    AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::autoPoseSet, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live in your
                                         // Constants class
            new PIDConstants(7.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(7.0, 0.0, 0.0)
        ), // Rotation PID constants    
        GUIconfig,
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

  } catch (Exception e) {
    // Handle exception as needed
    System.out.println("PATHING - Could not initialize PathPlanner check for ~/deploy/pathplanner/settings.json");
    e.printStackTrace();
    System.out.println("PATHING - End of stack trace --------------");
  }


  }

  public void simulationInit() {
    // WIP placeholder
    // motor/inertia models
  }

  @Override
  public void simulationPeriodic() {
    // WIP
  }

  public SwerveDriveOdometry getOdometry() {
    return m_odometry;
  }

  public SwerveModuleMK3 getMK3(int modID) {
    if ((modID < 0) || (modID > modules.length - 1))
      return null;
    return modules[modID];
  }

  // called by visionPoseEstimator
  public boolean useVisionRotation() {
    return visionPoseUsingRotation;
  }

  public boolean useVisionPose() {
    return visionPoseEnabled;
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
  }

  // these pose accessor are used in cmds, pose value supplied by pose estimator
  public Pose2d getPose() {
    return m_pose;
  }

  public void printPose() {
    Pose2d pose = getPose();
    System.out.println("***POSE X:" + pose.getX() +
        ", Y:" + pose.getY() +
        ", Rot:" + pose.getRotation().getDegrees());
  }

  // These are used by some commands.
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

  // reset angle to be zero, but retain X and Y; takes a Rotation2d object
  public void resetAnglePose(Rotation2d rot) {
    m_pose = new Pose2d(getPose().getX(), getPose().getY(), rot);
    m_odometry.resetPosition(sensors.getRotation2d(), meas_pos, m_pose); // updates gryo offset
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
   * stop() zero the current state's velocity component and leave angles
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

  public double getDistanceToTranslation(Translation2d targetTranslation) {
    return Math.sqrt(
        Math.pow(getPose().getTranslation().getX() - targetTranslation.getX(), 2)
            + Math.pow(getPose().getTranslation().getY() - targetTranslation.getY(), 2));

  }

}