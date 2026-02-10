package frc.robot2024.subsystems;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Either we have the same setup everytime
 * Otherwise, call IntakeDefaultPos to go to limit switch pos
 * this then sets the limit switch pos to a number (probably 0)
 * every time
 */

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.AlternateEncoderConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot2024.Constants.CAN;
import frc.robot2024.Constants.DigitalIO;
import frc.lib2202.command.WatcherCmd;
import frc.lib2202.util.NeoServo;
import frc.lib2202.util.PIDFController;

public class Intake extends SubsystemBase {
  //angle constants for commands
  public static final double UpPos = 2.0; // [deg]
  public static final double ShootingPos = 20.0; // [deg]
  public static final double DownPos = 108.0; // [deg]
  public static final double TravelUp = 180.0; // [deg/s]
  public static final double TravelDown = 170.0; // [deg/s]
  
  //roller constants for commands TODO set these consts 
  public static final double RollerMaxSpeed = 42.7; //[cm/s]
  public static final double RollerEjectSpeed = -42.7; //[cm/s]
  public static final double RollerAmpSpeed = -40.0; //[cm/s]

  //angle constants - internal
  final double AngleGearRatio = 405.0; // Gear ratio from gearbox stack

  // External encoder used
  // https://www.revrobotics.com/rev-11-1271/
  // https://docs.revrobotics.com/sparkmax/operating-modes/using-encoders/alternate-encoder-mode
  static final int Angle_kCPR = 8192; // alt encoder angle [counts per rotation]

  // roller constants
  final double RollerVelTol = 0.1; //[cm/s]
  final static double kff = 1.0/RollerMaxSpeed; //TODO find it
  final double wheelGearRatio = 1.0/(7.0*3.0); // [cm/s] - 7x3 on motor, 4.4 big gears

  // Intake devices 
  final NeoServo angle_servo;
  // PIDFController hwAngleVelPID = new PIDFController(/* 0.002141 */0.010, 0.00003, 0.0, /* 0.00503 */0.0045); //internal vel
  PIDFController hwAngleVelPID = new PIDFController(0.0050, 0.0, 0.0, 0.0075); // 3/2/24 improved vel loop match (new
                                                                                  // encoder)
  /* tune inner (hw/vel) go up until oscilation then divide by 2 should be close, then tune position */
  final PIDController anglePositionPID = new PIDController(7.0, 1.0, 0.0); 

  // TODO - add PIDs (hw and position for alpha bot), use when altEncoder is true

  // Intake roller motor
  final SparkMax intakeMtr = new SparkMax(CAN.INTAKE_MTR, SparkMax.MotorType.kBrushless);
  final SparkMaxConfig intakeMtrCfg = new SparkMaxConfig();
  final PIDFController intakeVelPID = new PIDFController(0.008, 0.000012, 0.0, kff); // tuned 3/19 (pls check NR)
  final double IntakeIZone = 5.0; //[cm/s] error IZone restriction

  final SparkClosedLoopController intakeMtrPid;
  final RelativeEncoder intakeMtrEncoder;
  double cmdVelocity = 0.0; //[cm/s] latest commanded velocity

  // lightgate tell us when we have a game piece (aka a Note) in roller
  final DigitalInput lightgate = new DigitalInput(DigitalIO.Intake_LightGate);

  // limit switch - only on alpha bot
  // DigitalInput limitSwitchUp = new DigitalInput(DigitalIO.IntakeIsUp);
  // DigitalInput limitSwitchDown = new DigitalInput(DigitalIO.IntakeIsDown);

  // Note State variables
  boolean hasNote = false; // true when Intake has Note

  final boolean altencoder;

  /*
   * Intake 
   * 
   * @param altEncoder - true for alphaBot, false for Elvis
   * 
   */
  public Intake(boolean altEncoder) { 
    final int STALL_CURRENT = 30; // [amp] staying with 550
    final int FREE_CURRENT = 20; // [amp]
    double angMaxVel = 200.0; // [deg/s]
    final double angMaxAccel = 200.0; // [deg/s^2] (likely not used in servo until smart profile is enabled)
    final double angPosTol = 2.0; // [deg]
    final double angVelTol = 1.0; // [deg/s]
    
    this.altencoder = altEncoder;
  
    //intake roller constants
    final double wheel_radius = 1.55*2.54; //in --> [cm]
    final double conversionFactor = wheel_radius*wheelGearRatio;

    // servo controls angle of intake arm, setup for velocity mode on brushless
    // motor
    // using hack (alt encoder used as flag for elvis) - strange inversion
    angle_servo = new NeoServo(CAN.ANGLE_MTR,
        anglePositionPID, hwAngleVelPID,
        !altEncoder, ClosedLoopSlot.kSlot0);

    // use velocity control on intake motor
    intakeMtr.clearFaults();
    //intakeMtr.restoreFactoryDefaults();  //see mtr.configure()
    // alt encoder false for beta & beta inverted from alpha
    intakeMtrCfg
      .inverted(!altEncoder)
      .idleMode(IdleMode.kCoast);
    intakeMtrCfg.encoder
      .positionConversionFactor(conversionFactor)
      .velocityConversionFactor(conversionFactor / 60.0); // min to sec
    
    intakeVelPID.setIZone(IntakeIZone);

    // copy hw pid setting for intake roller to the intakeMtrPid
    intakeVelPID.copyTo(intakeMtr, intakeMtrCfg, ClosedLoopSlot.kSlot0);
    intakeMtr.configure(intakeMtrCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeMtrPid = intakeMtr.getClosedLoopController();
    intakeMtrEncoder = intakeMtr.getEncoder();
    
    // Servo setup for angle_servo
    //should be done in NeoServo  //hwAngleVelPID.copyTo(angle_servo.getController().getClosedLoopController(), 0);
    angle_servo.setConversionFactor(360.0 / AngleGearRatio) // [deg] for internal encoder behind gears
              // .setConversionFactor(360.0) // [deg] external encoder on arm shaft
              .setSmartCurrentLimit(STALL_CURRENT, FREE_CURRENT)
              .setVelocityHW_PID(angMaxVel, angMaxAccel)
              .setTolerance(angPosTol, angVelTol)
              .setMaxVelocity(angMaxVel)
              .burnFlash();

    // Add external Encoder for position, but use Vel mode on inner loop
    if (altEncoder) {
      angle_servo.addAltPositionEncoder(Type.kQuadrature, Angle_kCPR, 360.0);
    }
    // power on
    setAnglePosition(UpPos);
    angle_servo.setClamp(UpPos, DownPos + 5.0);
  }

  // Used for beta (new bot/elvis)
  public Intake() {
    this(false);
  }

  /**
   * Set the intake's speed to given value
   * @param speed [cm/s]
   */
  public void setIntakeSpeed(double speed) {      
    intakeMtrPid.setSetpoint(speed, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    // clear any windup on stop
    if (speed == 0.0)
      intakeMtrPid.setIAccum(0.0);
    cmdVelocity = speed;
  }

  public boolean senseNote() {
    return !lightgate.get();
  }

  public boolean hasNote() {
    return hasNote;
  }

  public double getIntakeRollerSpeed() {
    return intakeMtrEncoder.getVelocity();
  }

  /* [deg] */
  public void setAngleSetpoint(double position) {
    angle_servo.setSetpoint(position);
  }

  /* [deg] */
  public double getAngleSetpoint() {
    return angle_servo.getSetpoint();
  }

  /* [deg] */
  public double getAnglePosition() {
    return angle_servo.getPosition();
  }

  /*
   * sets anglePosision to given value, doesn't move the intake.
   * Useful for calibration with limits or power up.
   */
  public void setAnglePosition(double pos) {
    angle_servo.setPosition(pos);
  }

  public void setMaxVelocity(double velLimit) {
    angle_servo.setMaxVelocity(velLimit);
  }

  /*
   * [deg/s]
   * Switches angle servo to velcoity mode
   * Used for test and calibration.
   */
  public void setAngleVelocity(double speed) {
    angle_servo.setVelocityCmd(speed);
  }

  public boolean rollerAtVelocity() {
    return Math.abs(getRollerCmdVelocity() - intakeMtrEncoder.getVelocity() ) <= RollerVelTol;
  }

  public double getAngleVelocity() {
    return angle_servo.getVelocity();
  }

  public double getRollerCmdVelocity() {
    return cmdVelocity;
  }

  public double getAngleSpeed() {
    return angle_servo.getVelocity();
  }

  public boolean angleAtSetpoint() {
    return angle_servo.atSetpoint(); // are we there yet?
  }

  // public boolean atForwardLimitSwitch() {
  //   return !limitSwitchDown.get();
  // }

  // public boolean atReverseLimitSwitch() {
  //   return !limitSwitchUp.get();
  // }

  public Command getWatcher() {
    return new IntakeWatcherCmd();
  }

  public void setHasNote(boolean state) {
    // if we ever lose a note, call this
    hasNote = state;
  }

  public void periodic() {
    this.angle_servo.periodic(); // do child objects first
    // no need for edge detect on Elvis, gate covers horizantal region of intake roller
    hasNote = senseNote();
  }

  // Account for minor differences between the different intake versions
  public double getDownAngle() {
    return altencoder  ? 91.0 : Intake.DownPos;
  }

  /*
   * Watcher commmand puts network table data for intake.
   * 
   */
  class IntakeWatcherCmd extends WatcherCmd {
    NetworkTableEntry nt_hasNote;
    NetworkTableEntry nt_angleVel;
    NetworkTableEntry nt_kP;
    NetworkTableEntry nt_kI;
    NetworkTableEntry nt_kD;
    NetworkTableEntry nt_wheelVel;
    NetworkTableEntry nt_anglePos;
    NetworkTableEntry nt_angleCmd;
    NetworkTableEntry nt_forwardLimit;
    NetworkTableEntry nt_reverseLimit;
    NetworkTableEntry nt_reverseLimitSwitchEnabled;
    NetworkTableEntry nt_forwardLimitSwitchEnabled;
    NetworkTableEntry nt_rollerCmdSpeed;
    NetworkTableEntry nt_rollerSpeed;
    NetworkTableEntry nt_lightgate;
    NetworkTableEntry nt_senseNote;

    @Override
    public String getTableName() {
      return Intake.this.getName();
    }

    public void ntcreate() {
      NetworkTable table = getTable();
      nt_lightgate = table.getEntry("dio_Note");
      nt_angleVel = table.getEntry("angleVel");
      nt_kP = table.getEntry("kP");
      nt_kI = table.getEntry("kI");
      nt_kD = table.getEntry("kD");
      
      nt_anglePos = table.getEntry("anglePos");
      nt_angleCmd = table.getEntry("anglePosCmd");
      nt_forwardLimit = table.getEntry("dio_LimitFwd");
      nt_reverseLimit = table.getEntry("dio_LimitRev");
  
      nt_rollerCmdSpeed = table.getEntry("rollerCmdSpeed");
      nt_rollerSpeed = table.getEntry("rollerSpeed");
      nt_hasNote = table.getEntry("hasNote");
      nt_senseNote = table.getEntry("senseNote");
    }

    public void ntupdate() {
      nt_lightgate.setBoolean(senseNote());
      nt_angleVel.setDouble(getAngleSpeed());
      nt_kP.setDouble(hwAngleVelPID.getP());
      nt_kI.setDouble(hwAngleVelPID.getI());
      nt_kD.setDouble(hwAngleVelPID.getD());
      
      nt_anglePos.setDouble(getAnglePosition());
      nt_angleCmd.setDouble(getAngleSetpoint());
      // nt_forwardLimit.setBoolean(atForwardLimitSwitch());
      // nt_reverseLimit.setBoolean(atReverseLimitSwitch());

      nt_rollerCmdSpeed.setDouble(getRollerCmdVelocity());
      nt_rollerSpeed.setDouble(getIntakeRollerSpeed());
      nt_hasNote.setBoolean(hasNote());
      nt_senseNote.setBoolean(senseNote());
    }
  } // watcher command

}
