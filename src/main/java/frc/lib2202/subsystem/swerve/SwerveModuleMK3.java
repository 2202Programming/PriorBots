package frc.lib2202.subsystem.swerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.Angle;
import frc.lib2202.builder.IRobotSpec;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.builder.RobotLimits;
import frc.lib2202.subsystem.swerve.config.ChassisConfig;
import frc.lib2202.util.ModMath;
import frc.lib2202.util.PIDFController;
import static frc.lib2202.Constants.DEGperRAD;

public class SwerveModuleMK3 {
  public final String NT_Name = "DT";

  // PID slot for angle and drive pid on SmartMax controller
  final ClosedLoopSlot kSlot = ClosedLoopSlot.kSlot0;

  private int frameCounter = 0;

  // Chassis config used for geometry and pathing math
  private final ChassisConfig cc;

  // Rev devices
  private final SparkMax driveMotor;
  private final SparkMaxConfig driveCfg;
  private final SparkMax angleMotor;
  private final SparkMaxConfig angleCfg;
  private final SparkClosedLoopController driveMotorPID;
  private final SparkClosedLoopController angleMotorPID; // sparkmax PID can only use internal NEO encoders
  private final RelativeEncoder angleEncoder; // aka internalAngle
  private final RelativeEncoder driveEncoder;
  
  // CTRE devices
  private final CANcoder absEncoder; // aka externalAngle (external to Neo/Smartmax)
  private double angleCmdInvert;

  /**
   * Warning CANCoder and CANEncoder are very close in name but very different.
   * 
   * CANCoder: CTRE, absolute position mode, +/- 180 CCW= positive CANEncoder:
   * RevRobotics, relative position only, must configure to CCW based on side &
   * gearing Continous positon so postion can be greater than 180 because it can
   * "infinitely" rotate. Cannot be inverted in Brushless mode, must invert motor
   * 
   */

  // NetworkTables
  String NTPrefix;

  // m_ -> measurements made every period - public so they can be pulled for
  // network tables...
  double m_internalAngle; // measured Neo unbounded [deg]
  double m_internalAngleMod;  //modulo
  double m_externalAngle; // measured CANCoder bounded +/-180 [deg]
  double m_velocity; // measured velocity [wheel's-units/s] [m/s]
  double m_position; // measure wheel positon for calibraiton [m]
  double m_angle_target; // desired angle unbounded [deg]
  double m_vel_target; // desired velocity [wheel's-units/s] [m/s]
  /**
   * SwerveModuleMK3 -
   * 
   * SmartMax controllers used for angle and velocity motors.
   * 
   * SmartMax Velocity mode is used to close the velocity loop. Units will match
   * the units of the drive-wheel-diameter.
   * 
   * Angle in degrees is controlled using position mode on the SmartMax. The angle
   * positon is not constrainted to +/- 180 degrees because the Neo has 32bit
   * float resolution, so we can just let the postion grow or shrink based on the
   * how many degrees we need to change. We could rotate 1000's of time without
   * going past the resolution of the SmartMax's position tracking. [deg]
   * 
   * Example: cmd_angle = 175 ==> 175 + (n * 360) where -Turns < n < Turns ==> ...
   * -545 == -185 == 175 == 535 == 895 ...
   * 
   * Minimum number of turns in one direction before we would have to consider
   * overflow: Turns = posBitResolution / encoder-counts Turns = 2^23 / (42*12.8)
   * = 15,603
   * 
   * Batteries will need changing before then.
   * 
   */
  public String myprefix;

  public SwerveModuleMK3(SparkMax driveMtr, SparkMax angleMtr, CANcoder absEnc,
      boolean invertAngleMtr, boolean invertAngleCmd, boolean invertDrive, String prefix) {
    driveMotor = driveMtr;
    angleMotor = angleMtr;
    absEncoder = absEnc;
    myprefix = prefix;

    IRobotSpec specs = RobotContainer.getRobotSpecs();
    RobotLimits limits = specs.getRobotLimits();

    // cc is the chassis config for all our pathing math
    cc = specs.getChassisConfig();

    /********************* not needed with new lib2025 
    // Always restore factory defaults at least once for new sparks - it removes gremlins
    driveMotor.restoreFactoryDefaults(specs.burnFlash());
    sleep(specs.burnFlash() ? 1000 : 0); // only need if flash is true
    angleMotor.restoreFactoryDefaults(specs.burnFlash());
    sleep(specs.burnFlash() ? 1000 : 0); // only need if flash is true
      *********************************/
    
      // account for command sign differences if needed
    angleCmdInvert = (invertAngleCmd) ? -1.0 : 1.0;
    
    driveCfg = new SparkMaxConfig();
    // Drive Motor config
    driveCfg.inverted(invertDrive)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(limits.driveStallAmp, limits.freeAmp)
            .encoder   // set driveEncoder to use units of the wheelDiameter, meters
              .positionConversionFactor(Math.PI * cc.wheelDiameter / cc.kDriveGR) // mo-rot to wheel units
              .velocityConversionFactor((Math.PI * cc.wheelDiameter / cc.kDriveGR) / 60.0); // mo-rpm wheel units
    
    // finish pid and config 
    cc.drivePIDF.copyTo(driveMtr, driveCfg, kSlot); // velocity mode
    driveMotor.configure(driveCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    driveMotorPID = driveMotor.getClosedLoopController();
    driveEncoder = driveMotor.getEncoder();   
    driveEncoder.setPosition(0.0);
    
    sleep(100);
    // Angle Motor config
    angleCfg = new SparkMaxConfig();
    angleCfg.inverted(invertAngleMtr)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(limits.angleStallAmp, limits.freeAmp)
            .encoder // set angle endcoder to return values in deg and deg/s
              .positionConversionFactor(360.0 / cc.kSteeringGR) // mo-rotations to degrees
              .velocityConversionFactor(360.0 / cc.kSteeringGR / 60.0); // rpm to deg/s  
    //finish angle controller
    cc.anglePIDF.copyTo(angleMtr, angleCfg, kSlot); // position mode
    angleMotor.configure(angleCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    angleMotorPID = angleMotor.getClosedLoopController();
    angleEncoder = angleMotor.getEncoder();

    NTPrefix = "/MK3-" + myprefix;
    NTConfig();
    calibrate();
  }

  // PID accessor for use in Test/Tune Commands
  public void setDrivePID(PIDFController temp) {
    temp.copyTo(driveMotor, driveCfg, kSlot);
  }

  public void setAnglePID(PIDFController temp) {
    temp.copyTo(angleMotor, angleCfg, kSlot);
  }

  /**
   * calibrate() - aligns Neo internal position with absolute encoder. This needs
   * to be done at power up, or when the unbounded encoder gets close to its
   * overflow point.
   * 
   * CanCoder note:  
   *    absolutePosition -  +-.5 range (-=180 deg) and includes the mag offset correction
   *    normal position - has own offset, created by setting its position, so there are two offsets.
   *                    - position (user offset) is calculated from diff of absPos and set value (I think)
   *                    - normal position will grow to count total rotations, so mod math is needed
   *                      if you want to use it on +/- 180 range.
   *  
   */
  void calibrate() {
    // read absEncoder position, set internal angleEncoder to that value adjust for cmd inversion.
    StatusSignal<Angle> abspos_deg = absEncoder.getAbsolutePosition().waitForUpdate(1.0);
    double cc_pos = angleCmdInvert * abspos_deg.getValue().in(BaseUnits.AngleUnit) * DEGperRAD;
    double after = -9999.0;
    double before = -9999.9;
    int count = 0;
    REVLibError angEncErr = REVLibError.kError;
    while ( (angEncErr != REVLibError.kOk) && Math.abs(after - cc_pos ) > 0.1 && count < 5)
    {
      before = angleEncoder.getPosition();
      angEncErr =  angleEncoder.setPosition(cc_pos);
      sleep(100); // sparkmax gremlins
      after = angleEncoder.getPosition();
      System.out.println("  calibrate("+this.myprefix+") pass("+count+ ") absEnc= " +  cc_pos + 
        " before="+before +"  after=" + after );
      count++;
    }
    realityCheckSparkMax(cc_pos, after);
  }

  void realityCheckSparkMax(double angle_cancoder, double internal_angle) {
    boolean result = true;
    var d_enc =  driveMotor.configAccessor.encoder;
    if (Math.abs(
      d_enc.getPositionConversionFactor() - Math.PI * cc.wheelDiameter / cc.kDriveGR) > 0.1) {
      System.out.println("*** ERROR *** " + myprefix + " position conversion factor incorrect for drive");
      System.out.println("Expected Position CF: " + Math.PI * cc.wheelDiameter / cc.kDriveGR);
      System.out.println("Returned Position CF: " + d_enc.getPositionConversionFactor());
      result = false;
    }
    if (Math.abs(d_enc.getVelocityConversionFactor() - Math.PI * cc.wheelDiameter / cc.kDriveGR / 60.0) > 0.1) {
      System.out.println("*** ERROR *** " + myprefix + " velocity conversion factor incorrect for drive");
      System.out.println("Expected Vel CF: " + Math.PI * cc.wheelDiameter / cc.kDriveGR / 60.0);
      System.out.println("Returned Vel CF: " + d_enc.getVelocityConversionFactor());
      result = false;
    }
    var a_enc = angleMotor.configAccessor.encoder;
    if (Math.abs(a_enc.getPositionConversionFactor() - (360.0 / cc.kSteeringGR)) > 0.1) {
      System.out.println("*** ERROR *** " + myprefix + " position conversion factor incorrect for angle");
      System.out.println("Expected Angle Pos CF: " + 360.0 / cc.kSteeringGR);
      System.out.println("Returned Angle Pos CF: " + a_enc.getPositionConversionFactor());
      result = false;
    }
    if (Math.abs(a_enc.getVelocityConversionFactor() - (360.0 / cc.kSteeringGR / 60)) > 0.1) {
      System.out.println("*** ERROR *** " + myprefix + " velocity conversion factor incorrect for angle");
      System.out.println("Expected Angle Vel CF: " + (360.0 / cc.kSteeringGR / 60));
      System.out.println("Returned Angle Vel CF: " + a_enc.getVelocityConversionFactor());
      result = false;
    }
    if (Math.abs(angle_cancoder - internal_angle) > 0.1) {
      System.out.println("*** ERROR *** " + myprefix + " angle encoder save error");
      System.out.println("Expected internal angle: " + angle_cancoder);
      System.out.println("Returned internal angle: " + internal_angle);
      result = false;
    }
    if (result) {
      System.out.println(myprefix + " passed reality checks.");
    }
    return;
  }

  /*******************************************lib2025 removed 
  // _set<> for testing during bring up.
   void _setInvertAngleCmd(boolean invert) {
    angleCmdInvert = (invert) ? -1.0 : 1.0;
    calibrate();
  }

  void _setInvertAngleMotor(boolean invert) {
    angleMotor.setInverted(invert);
  }

  void _setInvertDriveMotor(boolean invert) {
    driveMotor.setInverted(invert);
  }
  **************************************************************/

  /**
   * setNTPrefix - causes the network table entries to be created and updated on
   * the periodic() call.
   * 
   * Use a short string to indicate which MK unit this is.
   * 
   * public SwerveModuleMK3 setNTPrefix(String prefix) { NTPrefix = "/MK3-" +
   * prefix; myprefix = prefix; NTConfig(); return this; }
   */

  public String getNTPrefix() {
    return NTPrefix;
  }

  public void periodic() {
    // measure everything at same time; these get updated every cycle
    m_internalAngle = angleEncoder.getPosition() * angleCmdInvert;
    m_internalAngleMod = ModMath.fmod360(m_internalAngle );
    m_velocity = driveEncoder.getVelocity();
    m_position = driveEncoder.getPosition();
    m_externalAngle = absEncoder.getAbsolutePosition().getValueAsDouble() * 360.0;
    // these are for human consumption, update slower
    if (frameCounter++ > 10) {      
      NTUpdate();
      frameCounter = 0;
    }
  }

  public void simulationPeriodic() {

  }

  /**
   * This is the angle being controlled, so it should be thought of as the real
   * angle of the wheel.
   * 
   * @return SmartMax/Neo internal angle in Rotation2d object [rad]
   */
  public Rotation2d getAngleRot2d() {
    return Rotation2d.fromDegrees(m_internalAngle);
  }

  public double getAngle() {
    return m_internalAngle;
  }

  /**
   * External Angle is external to the SmartMax/Neo and is the absolute angle
   * encoder.
   * 
   * At power-up, this angle is used to calibrate the SmartMax PID controller.
   * 
   */
  public Rotation2d getAngleExternalRot2d() {
    return Rotation2d.fromDegrees(m_externalAngle);
  }

  public double getAngleExternal() {
    return m_externalAngle;
  }

  /**
   * 
   * @return velocity wheel's units [m]
   */
  public double getVelocity() {
    return m_velocity;
  }

  /**
   * 
   * @return velocity wheel's units [m]
   */
  public double getPosition() {
    return m_position;
  }

  // Expose the position with wpi class, [m], [rad]
  public SwerveModulePosition getSMPosition() {
    return new SwerveModulePosition(m_position, getAngleRot2d());
  }

  /**
   * Set the speed + rotation of the swerve module from a SwerveModuleState object
   * 
   * @param desiredState - A SwerveModuleState representing the desired new state
   *                     of the module
   */
  public void setDesiredState(SwerveModuleState state) {
     // should favor reversing direction over turning > 90 degrees
    // 1/4/2025 
    //this call was deprecated, use instance version preferred
    /* 
    SwerveModuleState m_state = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(m_internalAngle));
    state = m_state; // uncomment to use optimized angle command
    */
    state.optimize(Rotation2d.fromDegrees(m_internalAngle));
    
    
    // use position control on angle with INTERNAL encoder, scaled internally for
    // degrees
    m_angle_target = state.angle.getDegrees();

    // figure out how far we need to move, target - current, bounded +/-180
    double delta = ModMath.delta360(m_angle_target, m_internalAngle);
    // if we aren't moving, keep the wheels pointed where they are
    delta = (Math.abs(state.speedMetersPerSecond) < .01) ? 0.0 : delta;

    // now add that delta to unbounded Neo angle, m_internal isn't range bound
    angleMotorPID.setReference(angleCmdInvert * (m_internalAngle + delta), ControlType.kPosition);

    //save target vel for plots
    m_vel_target =state.speedMetersPerSecond;
    
    // use velocity control
    driveMotorPID.setReference(state.speedMetersPerSecond, ControlType.kVelocity);
  }

  /**
   * Network Tables data
   * 
   * If a prefix is given for the module, NT entries will be created and updated
   * on the periodic() call.
   * 
   */
  private NetworkTable table;
  private NetworkTableEntry nte_angle;
  private NetworkTableEntry nte_angleMod;
  private NetworkTableEntry nte_external_angle;
  private NetworkTableEntry nte_velocity;
  private NetworkTableEntry nte_position;
  private NetworkTableEntry nte_angle_target;
  private NetworkTableEntry nte_vel_target;
  private NetworkTableEntry nte_motor_current;
  private NetworkTableEntry nte_applied_output;

  void NTConfig() {
    // direct networktables logging
    table = NetworkTableInstance.getDefault().getTable(NT_Name);
    nte_angle = table.getEntry(NTPrefix + "/angle");
    nte_angleMod = table.getEntry(NTPrefix + "/angleMod");
    nte_external_angle = table.getEntry(NTPrefix + "/angle_ext");
    nte_velocity = table.getEntry(NTPrefix + "/velocity");
    nte_angle_target = table.getEntry(NTPrefix + "/angle_target");
    nte_vel_target = table.getEntry(NTPrefix + "/velocity_target");
    nte_position = table.getEntry(NTPrefix + "/position");
    nte_motor_current = table.getEntry(NTPrefix + "/motor_current");
    nte_applied_output = table.getEntry(NTPrefix + "/applied_output");
  }

  void NTUpdate() {
    if (table == null)
      return; // not initialized, punt
    nte_angle.setDouble(m_internalAngle);
    nte_angleMod.setDouble(m_internalAngleMod);
    nte_external_angle.setDouble(m_externalAngle);
    nte_velocity.setDouble(m_velocity);
    nte_position.setDouble(m_position);
    nte_angle_target.setDouble(m_angle_target);
    nte_vel_target.setDouble(m_vel_target);
    nte_motor_current.setDouble(driveMotor.getOutputCurrent());
    nte_applied_output.setDouble(driveMotor.getAppliedOutput());
  }

  public static void sleep(long ms) {
    try {
      Thread.sleep(ms);
    } catch (Exception e) {
    }
  }

  SparkClosedLoopController getDrivePID() {
    return driveMotorPID;
  }

  SparkClosedLoopController getAnglePID() {
    return angleMotorPID;
  }

  public void setBrakeMode() {
    SparkMaxConfig cfg = new SparkMaxConfig();
    cfg.idleMode(IdleMode.kBrake);
    driveMotor.configureAsync(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    angleMotor.configureAsync(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void setCoastMode() {
    SparkMaxConfig cfg = new SparkMaxConfig();
    cfg.idleMode(IdleMode.kCoast);
    driveMotor.configureAsync(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    angleMotor.configureAsync(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

}