// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.FeedbackSensor; 
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib2202.command.WatcherCmd;
import frc.lib2202.util.NeoServo;
import frc.lib2202.util.PIDFController;
import frc.robot2025.Constants.CAN;
import frc.robot2025.Constants.DigitalIO;


public class Elevator_Subsystem extends SubsystemBase {
  /** Creates a new Elevator_Subsystem. */

  /*
   * All placeholders 2/5/25
   *  maybe add algae level
   */
  public enum Levels {
    LOne(0.0), 
    LTwo(27.5), 
    LThree(67.5), 
    LFour(146.5),
    PickUp(0.0),
    PowerUp(0.0);
    public double height;

    private Levels(double height) {
      this.height = height;
    }
  }; 

  private final PIDController positionPid;   //software on rio
  private final PIDFController velocityPid;  //values copied to spark, done on hw controller
  private NeoServo servo; 
  private SparkFlex followMotor;
  private SparkFlexConfig followMotorConfig;
  private SparkClosedLoopController cl_ctrl; 

  final DigitalInput zeroLimitSwitch = new DigitalInput(DigitalIO.ElevatorZeroLS);
  final int STALL_CURRENT = 45;//60;
  final int FREE_CURRENT = 5;
  final double elevatorMaxVel = 175.0; // [cm/s] rpm
  final double elevatorMaxAccel = 100.0; // [cm/s^2]  servo may not enforce yet
  final double elevatorPosTol = 2.5;  // [cm]  -dpl with current gearing this is about it.
  final double elevatorVelTol = 0.5;  // [cm]
  final double maxPos = 149.0; // [cm]
  final double minPos = 0.0;  // [cm]
  final double initPos = 0.0;  // [cm]  initial power up position for relative encoders
  final boolean motors_inverted = false;

  private final double gearRatio = 1.0/4.67; // [out turns]/[mtr turns]
  private final double chainRatio = 1.0;    // [out/in] chain in/out 
  private final double pitchDiameter = 1.76;   // [cm]   
  private final double sprocket_circumference = 5.529;
  private final double stagesRatio = 1.0;   // [out/in] 
  // cf_spec - TODO not used yet, should workout to what was measured/corrected
  public final double cf_spec = gearRatio * stagesRatio * chainRatio * pitchDiameter * Math.PI * sprocket_circumference ;
  public final double cf = 8.75;  // 8.9026[cm/mtr-rot]  // ad-hoc measured 2/17/25, 12.627857 bfre
 
  
  public Elevator_Subsystem() {
    //init pid constant holders
    //software position pid - run in servo's periodic to control elevator position
    positionPid = new PIDController(5.0, 0.0005, 0.00);  //(7.0, 0.0005, 0.004
    positionPid.setIZone(3.0);
    //hardware velocity pidf - holds values to send to hw, not actually run825
    velocityPid = new PIDFController(0.00185, 0.000015, 0.0000, 1.0/565.0); //1.0/800 before, 565 is vortex Kv
    velocityPid.setIZone(10.0);
    
    //devices 
    servo = new NeoServo(CAN.ELEVATOR_MAIN, positionPid, velocityPid, motors_inverted, SparkFlex.class);
    followMotor = new SparkFlex(CAN.ELEVATOR_FOLLOW, MotorType.kBrushless); 

    //get closed-loop controller so we can monitor iAccum
    cl_ctrl = servo.getController().getClosedLoopController();
    cl_ctrl.setIAccum(0.0);
   
    // TODO - calibrate the cf so positions are accurate by using cf_spec 
    System.out.println("\tINITIAL CF=" + cf);
    System.out.println("\tCF_spec=" + cf_spec +" spec should come from gearRatio... fix it."); 
    
    //finish off the server setup
    servo
      .setConversionFactor(cf) //update with new values after testing
      .setTolerance(elevatorPosTol, elevatorVelTol)
      .setVelocityHW_PID(elevatorMaxVel, elevatorMaxAccel)
      .setSmartCurrentLimit(STALL_CURRENT, FREE_CURRENT)
      .setMaxVelocity(175.0)
      .setClamp(minPos, maxPos);
    
    //setup follower motor
    followMotorConfig = new SparkFlexConfig();
    followMotorConfig
      .inverted(motors_inverted)
      .idleMode(IdleMode.kBrake)
      .follow(CAN.ELEVATOR_MAIN) //motor 2 follows the servo's behavior
      .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder) 
          .outputRange(-1.0, 1.0);          
    //write the followMotor's config to hardware
    followMotor.configure(followMotorConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    // power up starting position of servo
    servo.setPosition(Levels.PowerUp.height);
    SmartDashboard.putNumber(KEY, 0.065);
   // servo.getWatcher();
  }
  String KEY = "ELEV_ARBFF";

  @Override
  public void periodic() {
    servo.periodic();
    double arbFF = SmartDashboard.getNumber(KEY, 0.0);
    servo.setArbFeedforward(arbFF);
  }

  
   
  public double getPosition() {
    return servo.getPosition();
  }

  // current access doesn't need to be exposed, putting on NT
  double getMainCurrent(){
    return servo.getController().getOutputCurrent();
  }

  public double getAccumI(){
    return cl_ctrl.getIAccum();
  }

  double getFollowCurrent() {
    return followMotor.getOutputCurrent();
  }
  public void setHeight (Levels level) {
    setHeight(level.height); 
  }

  public void setHeight (double height) {
    if (height > getPosition()) {
      servo.setMaxVelocity(175.0);
     // servo.setArbFeedforward(0.02);
    }
    else {  
      servo.setMaxVelocity(100.0);
    }
    servo.setSetpoint(height); 
  }

  public void setPosition(double pos){
    servo.setPosition(pos);
  }
  public double getSetpoint() {
    return servo.getSetpoint();
  }

  public double getVelocity() {
    return servo.getVelocity();
  }

  public void setVelocity(double vel) {
    if (vel > 0) {
      servo.setArbFeedforward(0.02);
    }
    else {  
      servo.setArbFeedforward(0.001);
    }
    servo.setVelocityCmd(vel);
  }

  public boolean atSetpoint() {
    return servo.atSetpoint();
  }

  public double getDesiredVelocity() {
    return servo.getVelocityCmd();
  }

  public WatcherCmd getWatcher() {
    return this.new ElevatorWatcherCmd();
  }

  public boolean atZeroLimit(){
    return !zeroLimitSwitch.get();
  }

   class ElevatorWatcherCmd extends WatcherCmd {
    NetworkTableEntry nt_cmdVel;
    NetworkTableEntry nt_measVel;
    NetworkTableEntry nt_desiredHeight;
    NetworkTableEntry nt_currentHeight;
    NetworkTableEntry nt_atHeight;
    NetworkTableEntry nt_mainCurrent;
    NetworkTableEntry nt_followCurrent;
    NetworkTableEntry nt_zeroLimitSwitch;
    NetworkTableEntry nt_iAccum;

    // add nt for pos when we add it
    @Override
    public String getTableName() {
      return Elevator_Subsystem.this.getName();
    }

    public void ntcreate() {
      NetworkTable table = getTable();
      nt_cmdVel = table.getEntry("Vel_cmd");
      nt_measVel = table.getEntry("Vel_meas");
      nt_desiredHeight = table.getEntry("Height_cmd");
      nt_currentHeight = table.getEntry("Height_meas");
      nt_atHeight = table.getEntry("atSetpoint");
      nt_mainCurrent = table.getEntry("amps_main");
      nt_followCurrent = table.getEntry("amps_follow");
      nt_zeroLimitSwitch = table.getEntry("zeroLimitSwitch");
      nt_iAccum = table.getEntry("iAccumX100");
    }

    public void ntupdate() {
      nt_cmdVel.setDouble(fmt2(getDesiredVelocity()));
      nt_measVel.setDouble(fmt2(getVelocity()));
      nt_desiredHeight.setDouble(fmt2(getSetpoint()));
      nt_currentHeight.setDouble(fmt2(getPosition()));
      nt_atHeight.setBoolean(atSetpoint());
      nt_mainCurrent.setDouble(fmt1(getMainCurrent()));
      nt_followCurrent.setDouble(fmt1(getFollowCurrent()));
      nt_zeroLimitSwitch.setBoolean(atZeroLimit());
      nt_iAccum.setDouble(fmt2(100.0*getAccumI()));
    }
  }

  
}
