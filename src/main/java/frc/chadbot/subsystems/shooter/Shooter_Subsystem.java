/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.chadbot.subsystems.shooter;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.chadbot.Constants.CAN;
import frc.chadbot.Constants.Shooter;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;


public class Shooter_Subsystem extends SubsystemBase  {
  public static final double USE_CURRENT_ANGLE = 0.0;

  /**
   * Creates a new Intake_Subsystem.
   * 
   * Use the WPI_TalonSRX wrapper around the lower level TalonSrx because it
   * implements the WPI SpeedController, Sendable.
   * 
   * 
   * Who When What DPL 2/09/2020 removed publics, use WPI_TalonSRX, Gear gain Also
   * removed some debug code. DPL 12/12/2020 setup for Kevin' PID UX, Moved motor
   * stuff to flywheel class invert flag for feedback, setInvert for motor
   * polarity.
   * 
   * DPL 12/15/2020 tested in lab, sort of worked but oscilated. Found Ki was 10x
   * what we tested 12/12 in DifferentialShooter branch. Changed, need to retest.
   *
   * DPL 1/30/2021 Tuned for new wheel layout. Added Izone in small region to zero
   * error Kd = at 80% of value that caused oscilation, want as large as possible
   * to resist speed change when ball is being launched. Kff = based on open loop
   * speed at max RPM
   * 
   * DPL 2/1/2021 moved constants to Constants.java split for different
   * upper/lower flywheel diameter added velocty and rotation as main control
   * inputs
   * 
   * DPL 1/30/2022 removed autoshoot state, do that in a command if we need it.
   * Cleaned up some naming conventions for public and private api.  Removed
   * public access to motors.
   * 
   * Terminology: PC stands for Power Cell, which is the same as Cargo or Ball in 2022.
   */

  private NetworkTable table;
  private NetworkTableEntry nt_upperRPM;
  private NetworkTableEntry nt_lowerRPM;
  private NetworkTableEntry nt_lowerGoal;
  private NetworkTableEntry nt_upperGoal;
  private NetworkTableEntry nt_upperRPMErr;
  private NetworkTableEntry nt_lowerRPMErr;
  private NetworkTableEntry nt_setpoint;
  
  // Flywheels 
  final FlyWheel  upper_shooter; 
  final FlyWheel  lower_shooter; 
  
  /**
   * ShooterSettings - simple struture to group the shooters settings.
   * 
   *   Rotations/sec is used as input, remember to convert to RADIAN/SEC
   *   to calculate speeds
   */
  public static class ShooterSettings 
  {
    public double vel;   // power cell ft/sec 
    public double rps;   // power cell rotations/sec 
    public double angle; // angle to set the shooter output
    public double velTol; // percent velocity Shooter must be at or below to fire

    public ShooterSettings(double vel, double rps, double angle, double velTol) {
      this.vel = vel; 
      this.rps = rps;
      this.angle = angle;
      this.velTol = velTol;
    }
    public ShooterSettings(double vel, double rps) {this(vel, rps, USE_CURRENT_ANGLE, Shooter.DefaultRPMTolerance);}
    public ShooterSettings(ShooterSettings s) {this(s.vel, s.rps, s.angle, s.velTol);}
    public ShooterSettings() {this(0.0, 0.0, USE_CURRENT_ANGLE, Shooter.DefaultRPMTolerance);}

    /**
     * equals - does a deep == on all terms
     * 
     * Note: == on floats means exact match, use with care. Only use is detecting changes
     * from a UX for sending new values to the device.
     * @param other  - other shooter settings to compare.
     * @return    true - exactly equal all terms
     *            false - any bit of difference use with care.
     */
    public boolean equals(ShooterSettings other) {
      return (this.vel == other.vel) &&
             (this.rps == other.rps) &&
             (this.angle == other.angle) &&
             (this.velTol == other.velTol);
    }
  }

  // All RPM are in Flywheel-RPM, not motor.
  FlyWheelRPM actual = new FlyWheelRPM();
  FlyWheelRPM target = new FlyWheelRPM();
  FlyWheelRPM error = new FlyWheelRPM();

  //Transfrom from [ w, V] [W_lower, W_upper]
  final Matrix<N2,N2> VelToRPM = new Matrix<>(Nat.N2(), Nat.N2() );
  Vector<N2> vel = new Vector<N2>(Nat.N2());
  
  //state variables
  private boolean m_readyToShoot = false;  
  ShooterSettings m_setpoint;     // reference to current shooter setpoint, angle, flywheel speeds
  
  /**
   * 
   */
  public Shooter_Subsystem() {

    upper_shooter = new FlyWheel(CAN.SHOOTER_UPPER_TALON, Shooter.upperFWConfig);
    lower_shooter = new FlyWheel(CAN.SHOOTER_LOWER_TALON, Shooter.lowerFWConfig);

    table = NetworkTableInstance.getDefault().getTable("Shooter");
    nt_upperRPM = table.getEntry("/UpperRPM/value");
    nt_lowerRPM = table.getEntry("/LowerRPM/value");
    nt_lowerGoal = table.getEntry("/LowerRPM/goal");
    nt_upperGoal = table.getEntry("/UpperRPM/goal");
    nt_upperRPMErr = table.getEntry("/UpperRPM/err");
    nt_lowerRPMErr = table.getEntry("/LowerRPM/err");
    nt_setpoint = table.getEntry("/TargetVel");
    
    // build out matrix to calculate FW RPM from [omega , Vel] for power cell
    VelToRPM.set(0, 0, Shooter.PCEffectiveRadius / Shooter.lowerFWConfig.flywheelRadius);
    VelToRPM.set(0, 1,  1.0 / Shooter.lowerFWConfig.flywheelRadius);
    VelToRPM.set(1, 0, -Shooter.PCEffectiveRadius / Shooter.upperFWConfig.flywheelRadius);
    VelToRPM.set(1, 1, 1.0 / Shooter.upperFWConfig.flywheelRadius);
    VelToRPM.times(0.5);  // common factor 1/2 //TODO I think this should be a factor of 2, not 1/2

    m_setpoint = Shooter.DefaultSettings;
  }

  @Override
  public void periodic() {
    //measure flywheel rpm and calculate our error 
    actual.set(upper_shooter.getRPM(), lower_shooter.getRPM());
    error.minus(target, actual);
 
    // monitor if the  shooter flywheel rpms to see if they are at setpoint
    double tol = m_setpoint.velTol;
    m_readyToShoot = ((Math.abs(error.upper) < (target.upper*tol)) &&
                      (Math.abs(error.lower) < (target.lower*tol)) );   
    log();
  }

  /**
   * Set the Shooter RPM goals from power cell velocity and rotation rate.  
   * 
   * Uses a transformation matrix
   * 
   *   [ VelToRPM<2,2>] * [w, vel ]^T =  [RPM_lower,  RPM_upper]^T
   *   
   *    @param ShooterSettings  
   *       @param pc_omeg_rps   Power Cell w = rotations / sec 
   *       @param pc_fps        Power Cell Vel = ft / sec
   * 
   *
   * @return FlyWheelRPM goal
   */
   FlyWheelRPM calculateGoals(ShooterSettings s) {
    final double radPerSec2revPerMin = 60 / (2.0*Math.PI);
    // order of input vector must match VelToRPM matrix order
    vel.set(0, 0, s.rps*2.0*Math.PI); //rad/s
    vel.set(1, 0, s.vel);             //ft/s  
    var omega = VelToRPM.times(vel).times(radPerSec2revPerMin);
    return new FlyWheelRPM(omega.get(0,0), omega.get(1,0));  
  }

  /**
   * setShooterSettings(ShooterSettings s)
   * 
   * Controls the setpoint of the shooter and all its possible 
   * settings. Null will reset to defaults.
   * 
   * @param s  - new settings, null resets to defaults
   */
  public void setShooterSettings(ShooterSettings s) {
    // default setting will shoot from any angle
    s = (s == null) ? Shooter.DefaultSettings : s;
    m_setpoint = s;
    nt_setpoint.setDouble(m_setpoint.vel);
  }
  
  public ShooterSettings getShooterSettings() { 
    return m_setpoint;
  }

  /**
   * spinup()
   * 
   * Preferred interface for controlling Flywheels
   * This will not shoot, that requires control of the belt and
   * perhaps some targing feedback.
   * 
   * Uses the m_setpoint settings
   * 
   * Use this interface to Warm-up the flywheels.
   * 
   */
  public void spinup() {
    // shooter will run at the target goals
    on(calculateGoals(m_setpoint));
  }

  /**
   * spinupShooter(ShooterSettings s)
   * Enables flywheels with a new setpoint.
   *   shorthand for:
   *      setShooterSettings(s);
   *      spinupShooter();
   * 
   * @param s   new setpoint for shooter
   */
  public void spinup(ShooterSettings s) {
     setShooterSettings(s);
    // shooter will run at the target goals
    on(calculateGoals(m_setpoint));
  }

  /**
   * Commands shooter flwwheels to target RPM
   * Motors are turned on with the given goals.
   * 
   * @param goals   (flywheel rpm goals)
   */
  public void on(FlyWheelRPM goals) {
    // save the targets for at goal calcs
    target.copy(goals);
    
    upper_shooter.setRPM(target.upper);
    lower_shooter.setRPM(target.lower);
  }

  /**
   * Shouldn't use this api, debugging only.
   */
  public void onPercent(double upperPct, double lowerPct) {
      upper_shooter.setPercent(upperPct); 
      lower_shooter.setPercent(lowerPct); 
  }

  public void off() {
    target.lower = 0;
    target.upper = 0;
    upper_shooter.setPercent(0.0);
    lower_shooter.setPercent(0.0);
  }

 
  public void getFlyWheelRPM(FlyWheelRPM ref) {
    ref.copy(actual);  //puts actual values into ref
  }

  public void getFlywheelTargetRPM(FlyWheelRPM ref) {
    ref.copy(target);  //puts values into given ref
  }

  
  /**
   * isReadyToShoot - public API, calculated in periodic from m_setpoint
   * @return  true  angle/speed ready
   *          false something is not ready
   */
  public boolean isReadyToShoot() {return m_readyToShoot;}

  
  public void log() {
    // Put any useful log message here, called about 10x per second
    nt_lowerRPM.setDouble(actual.lower);
    nt_upperRPM.setDouble(actual.upper);
    nt_lowerRPMErr.setDouble(error.lower);
    nt_upperRPMErr.setDouble(error.upper);
    nt_upperGoal.setDouble(actual.upper-error.upper);
    nt_lowerGoal.setDouble(actual.lower-error.lower);
  }

  public void setPIDUpper(double kP, double kI, double kD, double kF){
    upper_shooter.setPID(kP, kI, kD, kF);
  }

  public void setPIDLower(double kP, double kI, double kD, double kF){
    lower_shooter.setPID(kP, kI, kD, kF);
  }

  public double getlowerP(){
    return lower_shooter.getP();
  }

  public double getlowerI(){
    return lower_shooter.getI();
  }

  public double getlowerD(){
    return lower_shooter.getD();
  }

  public double getUpperP(){
    return upper_shooter.getP();
  }

  public double getUpperI(){
    return upper_shooter.getI();
  }

  public double getUpperD(){
    return upper_shooter.getD();
  }

  public double getUpperF(){
    return upper_shooter.getF();
  }
  public double getLowerF(){
    return upper_shooter.getF();
  }


}

