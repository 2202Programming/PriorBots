/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.chadbot.subsystems.shooter;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
//import com.ctre.phoenix.motorcontrol.can.TalonSRXPIDSetConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.lib2202.util.PIDFController;

/**
 * Flywheel handles motor and gearing for the shooter flywheels.
 * 
 */
public class FlyWheel {
  /**
   * PID Gains may have to be adjusted based on the responsiveness of control
   * loop. kF: 1023 represents output value to Talon at 100%, 7200 represents
   * Velocity units at 100% output.
   * 
   * Calculate Kf to get us close to desired speed and pid will fine tune it.
   * 
   * [FW-RPM] = flywheel rpm
   * [MU-100] = motor units per 100mS which is the controller's vel uint, [MU] for
   * short
   * 2000 RPM * 34.133 [MU-100ms]/[FW-RPM] = 68267 MU/FW-RPM
   * 
   * NOTE: if gearing is before the encoder, [MU-100] must account for gearing.
   * 
   * 2/6/21 Kff now calcualted from max FW RPM
   */

  public static class FlyWheelConfig {
    public PIDFController pid;
    public double maxOpenLoopRPM;
    public double gearRatio; // account for gearbox reduction to flywheel
    public boolean sensorPhase;
    public boolean inverted;
    public double flywheelRadius;
    public double FWrpe2MU; // FlywheelRPM to Motor-Units (includes gearing)
  };

/**
 * Encoder & controler details
 * Convert Target RPM to [motor-units/100ms] 4096 Units/Rev * Target RPM * 600 =
 * velocity setpoint is in units/100ms
 */
  final double kEncoder = 4096.0;   // encoder counts/rotation 
  final double kRP100 = 1.0/600.0;  // RPM to rev-per-100mS
  final double kMaxMO = 1023.0;     // max Motor output

  // Talon Slot stuff, we just use slot 0
  final int kPIDLoopIdx = 0;
  final int kTimeoutMs = 30;

  TalonSRXConfiguration srxconfig;
  WPI_TalonSRX motor; // this could be a generic motor controller...

  final double FWrpm2Counts; // flywheel RPM given motor-unit counts (f(gear, meas-period))
  final double MUCounts2FWrpm; // motor units (counts/100ms) to FW RPM (1/FWrpm2Counts)

  public FlyWheelConfig cfg;

  FlyWheel(int CAN_ID, FlyWheelConfig cfg) {
    srxconfig = new TalonSRXConfiguration();
    motor = new WPI_TalonSRX(CAN_ID);
    motor.setInverted(cfg.inverted);
    this.cfg = cfg;

    // flywheel constants RPM given encoder-unit counts (f(gear, meas-period))
    double FWCountsPerRot = kEncoder / cfg.gearRatio;     // encoder : fw shaft 
    FWrpm2Counts = FWCountsPerRot * kRP100;   // (counts/100mS)/RPM
    MUCounts2FWrpm = 1.0 / FWrpm2Counts;      // encoder units (counts/100mS) to FW RPM

    // use max rpm and max motor out to calculate kff
    double kff = kMaxMO / (cfg.maxOpenLoopRPM * FWrpm2Counts);
    cfg.pid.setF(kff);

    ErrorCode lasterr = motorConfig(cfg);
    if (lasterr.value != 0) {
      System.out.println("Flywheel motor error:" + lasterr.value + "  CANID=" + CAN_ID);
    }
  }

  ErrorCode motorConfig(FlyWheelConfig cfg) {
    /* Factory Default all hardware to prevent unexpected behaviour */
    motor.configFactoryDefault();

    // use the config to set all values at once
    cfg.pid.copyTo(motor, kPIDLoopIdx );
    motor.getAllConfigs(srxconfig);

    srxconfig.slot1 = srxconfig.slot0;
    motor.configAllSettings(srxconfig);

    /* Config sensor used for Primary PID [Velocity] */
    motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);
    motor.setSensorPhase(cfg.sensorPhase); // fix feedback direction
    motor.setNeutralMode(NeutralMode.Coast);

    /* Config the peak and nominal outputs */
    motor.configNominalOutputForward(0, kTimeoutMs);
    motor.configNominalOutputReverse(0, kTimeoutMs);
    motor.configPeakOutputForward(1, kTimeoutMs);
    motor.configPeakOutputReverse(-1, kTimeoutMs);
    return motor.getLastError();
  }

  /**
   * Gets RPM as measured at the flywheel
   * 
   * @return flywheel_rpm
   */
  public double getRPM() {
    double vel_mu = motor.getSelectedSensorVelocity(); // motor units
    return vel_mu * MUCounts2FWrpm;
  }

  public double getMotorOutputPercent() {
    return motor.getMotorOutputPercent();
  }

  public void setRPM(double fw_rpm) {
    double sp = fw_rpm * FWrpm2Counts;
    motor.set(ControlMode.Velocity, sp);
  }

  public void setPercent(double pct) {
    motor.set(ControlMode.PercentOutput, pct);
  }

  // This API is for testing/tuning only.
  public void setPID(double kP, double kI, double kD, double kF){
    cfg.pid.setPIDF(kP, kI, kD, kF);
    cfg.pid.copyTo(motor, kPIDLoopIdx);
  }

  public double getP(){
    return cfg.pid.getP();
  }

  public double getI(){
    return cfg.pid.getI();
  }

  public double getD(){
    return cfg.pid.getD();
  }
  public double getF() {
    return cfg.pid.getF(); 
  } 

} // FlyWheel
