// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
//2025 import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.FeedbackSensor; //2026 libs
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib2202.Constants;
import frc.lib2202.command.WatcherCmd;
import frc.lib2202.subsystem.UX.TrimTables.Trim;
import frc.lib2202.util.NeoServo;
import frc.lib2202.util.PIDFController;
import frc.robot2025.Constants.CAN;
import frc.robot2025.Constants.DigitalIO;

public class GroundIntake extends SubsystemBase {
  public static String TrimTableName = "GoundIntake";

  public enum Position {
    POWERUP(0.0, 0.0), // pwr up could be different from ZERO
    ZERO(0.0, 0.0),
    ALGAE_PICKUP(-60.0, 135.0),
    ALGAE_PLACE(-60.0, 100.0), // algae place
    ALGAE_REST(-60.0, 100.0),
    CORAL_PICKUP(-10.0, 135.0),
    CORAL_PLACE(-10.0, 45.0), // coral place
    CORAL_REST(-10.0, 45.0),
    FLOOR(0.0, 135.0);

    public double topval;
    public double btmval;

    private Position(double topval, double btmval) {
      this.topval = topval;
      this.btmval = btmval;
    }
  }

  // motor config constants
  final ClosedLoopSlot wheelSlot = ClosedLoopSlot.kSlot0;
  final int wheelStallLimit = 30;
  final int wheelFreeLimit = 5;
  final static double Kff = 0.005;
  final PIDFController wheelPIDF = new PIDFController(0.00025, 0.000000, 0.0, Kff);  // kp was 0.015                                                                         
  final static double wheelMtrGearRatio = 2.0; // 2 motor turns -> 1 wheel turn
  final LinearFilter wheelFilter = LinearFilter.singlePoleIIR(0.2, Constants.DT);

  final int StallCurrent = 40;
  final int FreeCurrent = 5;

  // servo config
  final NeoServo topServo;
  final NeoServo btmServo;
  final SparkMax wheelMtr;
  final RelativeEncoder wheelMtr_encoder;
  double wheel_current; //[amps]
  final double WheelCurrentTrip = 10.0;  //[amps] TBD
  double wheel_cmd=0.0;   //requested speed, to compare for game piece detect
  double wheel_speed; //measured speed
  int wheel_stall = 0;
  final int StallCountTrip = 3;
  boolean has_gamepiece = false;


  // gates for piece detection
  DigitalInput coralSwitch = new DigitalInput(DigitalIO.GroundIntakeHasCoral);
  DigitalInput algeSwitch = new DigitalInput(DigitalIO.GroundIntakeHasAlgae);

  final SparkMaxConfig wheelMtr_cfg;
  final SparkClosedLoopController wheelMtr_ctrl;
  //public static final double WheelMaxVolts = 5.0;

  PIDFController topHwAngleVelPID = new PIDFController(0.00075, 0.0, 0.0, 0.0013); // placeholder PIDs
  final PIDController topPositionPID = new PIDController(3.5, 0.0, 0.0);

  PIDFController btmHwAngleVelPID = new PIDFController(0.0007, 0.000001, 0.0, 0.0017);
  final PIDController btmPositionPID = new PIDController(3.5, 0.0007, 0.0); //kp = 2.5

  final double topServoGR = (1.0 / 150.0) * 360.0; // 150:1 gearbox reduction * 360 degrees / turn
  final double btmServoGR = (1.0 / 45.0) * 360.0; // 45:1 gearbox reduction * 360 degrees / turn

  final double topIRange = 1.0; // degrees, default is infinity

  // Where we are heading, use atSetpoint to see if we are there
  Position currentPos = Position.POWERUP;
  double top_cmd, btm_cmd;  //tracks last commanded positions
  Trim topTrim;
  double holdOffset;
  

  public GroundIntake() { 
    topTrim = new Trim(TrimTableName, "Top", this::trimChange, 0.0);

    topHwAngleVelPID.setIZone(25.0);
    topPositionPID.setIntegratorRange(-topIRange, topIRange);
    btmHwAngleVelPID.setIZone(25.0);
    topServo = new NeoServo(CAN.IntakeTop, topPositionPID, topHwAngleVelPID, true);
    btmServo = new NeoServo(CAN.IntakeBtm, btmPositionPID, btmHwAngleVelPID, true);
    wheelMtr = new SparkMax(CAN.IntakeWheel, MotorType.kBrushless);
    topServo.setConversionFactor(topServoGR)
      .setMaxVelocity(90.0) // [deg/s]
      .setTolerance(4.0, 0.5)
      .setSmartCurrentLimit(StallCurrent, FreeCurrent);
    btmServo.setConversionFactor(btmServoGR)
      .setMaxVelocity(120.0)
      .setTolerance(2.0, 0.5)
      .setSmartCurrentLimit(StallCurrent, FreeCurrent);
  
   
    // configure wheel motor
    wheelMtr_cfg = new SparkMaxConfig();
    wheelMtr_cfg.inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(wheelStallLimit, wheelFreeLimit).encoder
        .positionConversionFactor(wheelMtrGearRatio) // rotations
        .velocityConversionFactor(wheelMtrGearRatio / 60.0); // rps

    wheelMtr_cfg.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    // finish pid and config
    wheelPIDF.copyTo(wheelMtr, wheelMtr_cfg, wheelSlot); // velocity mode
    wheelMtr.configure(wheelMtr_cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    wheelMtr_ctrl = wheelMtr.getClosedLoopController();
    wheelMtr_encoder = wheelMtr.getEncoder();

    // Initialize our servo's power up conditions, does not change desired setpoint,
    // it just sets the encoder values directly
    topServo.setPosition(currentPos.topval);
    btmServo.setPosition(currentPos.btmval);

    // set our requested setpoint with our public api POWERUP
    setSetpoint(currentPos); // changes setpoints

    this.new GroundIntakeWatcher();

    //testing PIDF smartdashboard stuff
    //topHwAngleVelPID.setName("topGndIn");
  }

  public void setSetpoint(Position cmd) {
    currentPos = cmd;
    setSetpoint(cmd.topval, cmd.btmval);
  }

  public void setSetpoint(double top, double btm) {
    topServo.setArbFeedforward(0.0);
    btmServo.setArbFeedforward(0.0);
    topServo.setSetpoint(top + holdOffset);
    //track our targets for trimChange
    this.top_cmd = top;
    this.btm_cmd = btm;

    double trimedTop = topTrim.getValue(top) + holdOffset;  // testing trim api
    topServo.setSetpoint(trimedTop);
    btmServo.setSetpoint(btm);
  }

  // on a trim change, adjust the cmds
  // Returns bool because Void type seems bugged.
  Boolean trimChange() {
    setSetpoint(top_cmd, btm_cmd);
    return true;
  }


  public void debugBtmVelocity(double dir) {    
    double aff = 0.0;
    if(Math.abs(dir) > 0.5){
      aff = (dir > 0.0) ? 0.003 : -0.02;
    }
    if (dir== 0.0){
      aff = -0.02; // seemed to hold at 0 velocity
    }
    btmServo.setArbFeedforward(aff); 
    btmServo.setVelocityCmd(dir);
  }

  public void debugTopVelocity(double dir) {
    double aff = 0.0;
    if(Math.abs(dir) > 0.5){
      aff = (dir > 0.0) ? 0.003 : -0.02;
    }
    if (dir== 0.0){
      aff = 0.0;
    }
    topServo.setArbFeedforward(aff);
    topServo.setVelocityCmd(dir);
  }

  public void hold(double deg){
    this.holdOffset = deg;
  }

  public boolean isTopAtSetpoint() {
    return topServo.atSetpoint();
  }

  public boolean isBottomAtSetpoint() {
    return btmServo.atSetpoint();
  }

  public boolean isAtSetpoint() {
    return isTopAtSetpoint() && isBottomAtSetpoint();
  }

  public void setWheelSpeed(double speed) {
    wheel_cmd = speed;
    wheelMtr_ctrl.setSetpoint(speed, ControlType.kVelocity);
  }

  public void setWheelHold(double voltage){
    //wheel_cmd = 0.0;
    //wheelMtr_ctrl.setReference(voltage, ControlType.kVoltage);
    //setWheelSpeed(voltage); // HACK to test using slow speed to hold -er
    wheelMtr.set(voltage); // double hack for % pwr 
  }

  public double getTopPosition() {
    return topServo.getPosition();
  }

  public double getBtmPosition() {
    return btmServo.getPosition();
  }

  public boolean senseCoral() {
    return !coralSwitch.get();
  }

  public boolean senseAlgae() {
    return !algeSwitch.get();
  }


  public void setZero(){
    topServo.setPosition(0.0);
    btmServo.setPosition(0.0);
  }

  @Override
  public void periodic() {
    // protect against bad motion
    topServo.periodic();
    btmServo.periodic();

    wheel_current = wheelFilter.calculate(wheelMtr.getOutputCurrent());
    wheel_speed = wheelMtr_encoder.getVelocity();

    //count stalled wheel frames
    if (Math.abs(wheel_cmd) > 1.0 && Math.abs(wheel_speed) < 0.5){
      // wheel is stalled 
      wheel_stall++;
    } else {
      wheel_stall = 0;
    }

    // use wheel motor current/speed to detect gamepiece
    // when we don't have switches we expected, latch value until
    // cleard by a command.
    has_gamepiece = has_gamepiece || 
        (wheel_current > WheelCurrentTrip  &&
         wheel_stall > StallCountTrip) ;
  }

  //clears has_gamepiece latch
  public void clearGamePiece() {
    has_gamepiece = false;
    wheel_stall = 0;
  }

  public boolean getLatchedHasGamePiece() {
    return has_gamepiece;
  }

  public class GroundIntakeWatcher extends WatcherCmd {
    // Table Entries 
    NetworkTableEntry NT_topVelocity;
    NetworkTableEntry NT_btmVelocity;
    NetworkTableEntry NT_wheelVelocity;
    NetworkTableEntry NT_cmdWheelVelocity;
    NetworkTableEntry NT_hasCoral;
    NetworkTableEntry NT_hasAlgae;
    NetworkTableEntry NT_topPos;
    NetworkTableEntry NT_btmPos;
    NetworkTableEntry NT_topCmdVel;
    NetworkTableEntry NT_btmCmdVel;
    NetworkTableEntry NT_topCmdPos;
    NetworkTableEntry NT_btmCmdPos;
    NetworkTableEntry NT_topAtSetpoint;
    NetworkTableEntry NT_topGetIAccum;
    NetworkTableEntry NT_groundIntakeHasCoral;
    NetworkTableEntry NT_groundIntakeHasAlgae;
    NetworkTableEntry NT_has_gamepiece;
    NetworkTableEntry NT_wheel_current;

    public GroundIntakeWatcher() {
    }

    @Override
    public String getTableName() {
      return "Ground Intake";
    }

    @Override
    public void ntcreate() {
      NetworkTable MonitorTable = getTable();
      NT_topVelocity = MonitorTable.getEntry("top velocity");
      NT_btmVelocity = MonitorTable.getEntry("bottom velocity");
      NT_wheelVelocity = MonitorTable.getEntry("roller velocity");
      NT_hasCoral = MonitorTable.getEntry("hasCoral");
      NT_hasAlgae = MonitorTable.getEntry("hasAlgae");
      NT_topPos = MonitorTable.getEntry("top position");
      NT_btmPos = MonitorTable.getEntry("bottom position");
      NT_topCmdVel = MonitorTable.getEntry("top cmd velocity");
      NT_btmCmdVel = MonitorTable.getEntry("bottom cmd velocity");
      NT_topCmdPos = MonitorTable.getEntry("top cmd position");
      NT_btmCmdPos = MonitorTable.getEntry("bottom cmd position");
      NT_topAtSetpoint = MonitorTable.getEntry("is top at setpoint");
      NT_topGetIAccum = MonitorTable.getEntry("top IAccum");
      NT_cmdWheelVelocity = MonitorTable.getEntry("cmd wheel velocity");
      NT_has_gamepiece = MonitorTable.getEntry("hasGP");
      NT_wheel_current = MonitorTable.getEntry("wheel_current");
    }

    @Override
    public void ntupdate() {
      NT_topVelocity.setDouble(topServo.getVelocity());
      NT_btmVelocity.setDouble(btmServo.getVelocity());
      NT_wheelVelocity.setDouble(wheel_speed);
      NT_hasCoral.setBoolean(senseCoral());
      NT_hasAlgae.setBoolean(senseAlgae());
      NT_topPos.setDouble(topServo.getPosition());
      NT_btmPos.setDouble(btmServo.getPosition());
      NT_topCmdVel.setDouble(topServo.getVelocityCmd());
      NT_btmCmdVel.setDouble(btmServo.getVelocityCmd());
      NT_topCmdPos.setDouble(topServo.getSetpoint());
      NT_btmCmdPos.setDouble(btmServo.getSetpoint());
      NT_topAtSetpoint.setBoolean(isTopAtSetpoint());
      NT_topGetIAccum.setDouble(topServo.getController().getClosedLoopController().getIAccum());
      NT_has_gamepiece.setBoolean(has_gamepiece);
      NT_wheel_current.setDouble(wheel_current);
      NT_cmdWheelVelocity.setDouble(wheel_cmd);

      //call the pidf update so we can edit pids
      topHwAngleVelPID.NT_update();
      btmHwAngleVelPID.NT_update();  // must setup name - testing no-op 
    }
  } 

}
