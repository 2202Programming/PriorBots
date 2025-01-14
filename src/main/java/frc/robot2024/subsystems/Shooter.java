// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2024.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot2024.Constants.CAN;
import frc.robot2024.Constants.PCM1;
import frc.lib2202.command.WatcherCmd;
import frc.lib2202.util.PIDFController;

public class Shooter extends SubsystemBase {

  final SparkMax leftMtr = new SparkMax(CAN.SHOOTER_L, SparkMax.MotorType.kBrushless);
  final SparkMaxConfig leftMtrCfg = new SparkMaxConfig();
  final SparkMax rightMtr = new SparkMax(CAN.SHOOTER_R, SparkMax.MotorType.kBrushless);
  final SparkMaxConfig rightMtrCfg = new SparkMaxConfig();
  final SparkClosedLoopController hw_leftPid;
  final SparkClosedLoopController hw_rightPid;
  final RelativeEncoder leftEncoder;
  final RelativeEncoder rightEncoder;
  final double FACTOR = 1.0;
  final double left_kF = 1.0 / 5425.0; //5425 orig
  final double right_kF = 1.0 / 5500.0; //5100 orig
  public final double adjustment = 0.0;

  private DoubleSolenoid shooterAngle; // can be replaced w/ servo in derived class

  private double cmdLeftRPM;
  private double cmdRightRPM;
  private double measLeftRPM;
  private double measRightRPM;

  // left and right match pretty well.  25-50 rpm overshoot. 
  PIDFController leftPidConsts = new  PIDFController(0.000250, 0.0000002, 0.0, left_kF);  //slot 0  - normal
  PIDFController rightPidConsts = new PIDFController(0.000250, 0.0000002, 0.0, right_kF);
  PIDFController pidConsts_freeSpin = new PIDFController(0.0, 0.0, 0.0, 0.0);  //slot 1 - free spin

  public Shooter() {
    this(true);
  }

  public Shooter(boolean HasSolenoid) {
    motor_config(leftMtr, leftMtrCfg, leftPidConsts, false);
    motor_config(rightMtr, rightMtrCfg, rightPidConsts, true);

    hw_leftPid = leftMtr.getClosedLoopController();
    hw_rightPid = rightMtr.getClosedLoopController();
    leftEncoder = leftMtr.getEncoder();
    rightEncoder = rightMtr.getEncoder();
    if (HasSolenoid) {
      shooterAngle = new DoubleSolenoid(CAN.PCM1, PneumaticsModuleType.REVPH, PCM1.Forward, PCM1.Reverse);
      retract();
    }
  }

  @Override
  public void periodic() {
    measLeftRPM = leftEncoder.getVelocity();
    measRightRPM = rightEncoder.getVelocity();
  }

  public boolean isAtRPM(double tolerance) {
    return Math.abs(cmdLeftRPM - measLeftRPM) < tolerance
        && Math.abs(cmdRightRPM - measRightRPM) < tolerance;
  }


  public void setRPM(double leftRPM, double rightRPM) {
    // slot 0 --> normal op, slot 1 --> free spin
    ClosedLoopSlot slot = ClosedLoopSlot.kSlot0; // (leftRPM == 0.0 && rightRPM == 0.0) ? 1 : 0;
    if (leftRPM == 0.0 && rightRPM == 0.0) {
      hw_leftPid.setIAccum(0.0);
      hw_rightPid.setIAccum(0.0);
      slot = ClosedLoopSlot.kSlot1;
    } 
    
    hw_leftPid.setReference(leftRPM, ControlType.kVelocity, slot);
    hw_rightPid.setReference(rightRPM, ControlType.kVelocity, slot);
    cmdLeftRPM = leftRPM;
    cmdRightRPM = rightRPM;
  }

  /* 
   * Add compatibilty with ShooterServo
   */
  public void setAngleSetpoint(double angle) {
    // fake it as best as this one can...
    if (angle > 35.0)  deploy();
    else retract();
  }
/* 
   * Add compatibilty with ShooterServo
   */
  public void setExtensionPosition(double x) {
    //unused in base shooter
  }

  public void deploy() {
    shooterAngle.set(DoubleSolenoid.Value.kForward);
  }

  public void retract() {
    shooterAngle.set(DoubleSolenoid.Value.kReverse);
  }

  public WatcherCmd getWatcher() {
    return new ShooterWatcherCmd();
  }

  void motor_config(SparkMax mtr, SparkMaxConfig cfg, PIDFController hwPidConsts, boolean inverted) {
    mtr.clearFaults();
    cfg
      .idleMode(IdleMode.kBrake)
      .inverted(inverted);
  
    cfg.encoder
      .positionConversionFactor(FACTOR)
      .velocityConversionFactor(FACTOR /* / 60.0 */);
    
    cfg.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .iZone(750.0, ClosedLoopSlot.kSlot0);
    hwPidConsts.copyTo(mtr, cfg, ClosedLoopSlot.kSlot0);
   
    pidConsts_freeSpin.copyTo(mtr, cfg, ClosedLoopSlot.kSlot1); 
    cfg.closedLoop.iMaxAccum(0.0, ClosedLoopSlot.kSlot1);

    mtr.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  
  // Network tables
  class ShooterWatcherCmd extends WatcherCmd {
    NetworkTableEntry nt_cmdLeftMotorRPM;
    NetworkTableEntry nt_measLeftMotorRPM;
    NetworkTableEntry nt_cmdRightMotorRPM;
    NetworkTableEntry nt_measRightMotorRPM;
    NetworkTableEntry nt_kP;
    NetworkTableEntry nt_kF;

    // add nt for pos when we add it
    @Override
    public String getTableName() {
      return Shooter.this.getName();
    }

    public void ntcreate() {
      NetworkTable table = getTable();
      nt_cmdLeftMotorRPM = table.getEntry("cmdLeftMotorRPM");
      nt_measLeftMotorRPM = table.getEntry("measLeftMotorRPM");
      nt_cmdRightMotorRPM = table.getEntry("cmdRightMotorRPM");
      nt_measRightMotorRPM = table.getEntry("measRightMotorRPM");
      nt_kP = table.getEntry("kP");
      nt_kF = table.getEntry("kF");
    }

    public void ntupdate() {
      nt_cmdLeftMotorRPM.setDouble(cmdLeftRPM);
      nt_measLeftMotorRPM.setDouble(measLeftRPM);
      nt_cmdRightMotorRPM.setDouble(cmdRightRPM);
      nt_measRightMotorRPM.setDouble(measRightRPM);
      nt_kP.setDouble(leftMtr.configAccessor.closedLoop.getP(ClosedLoopSlot.kSlot0));  
      nt_kF.setDouble(leftMtr.configAccessor.closedLoop.getFF(ClosedLoopSlot.kSlot0));
    }
  }
}
