// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2024.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot2024.Constants.CAN;
import frc.lib2202.command.WatcherCmd;
import frc.lib2202.util.NeoServo;
import frc.lib2202.util.PIDFController;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  public final static double StartPosition = 0.0; //[cm]
  public final static double ExtendPosition = 27.0; //[cm]
  public final static double ClimbPosition = -3.5; //[cm]
  public final static double ClimbCalibrateVel = 2.0; //[cm/s]

  final double GearRatio = 1.0/25.0; // have 2 eventually probably lol
  double conversionFactor = 3.5 * 2.54 * GearRatio; //calculation here
  final double maxVel = 100.0; // placeholder. cm/s?
  final double maxAccel = 10.0; // placevholder cm/s^2
  double posTol = 0.25; // [cm]
  double velTol = .50; // [cm/s]
  final int STALL_CURRENT = 60; // placeholder // units?
  final int FREE_CURRENT = 30; // placeholder // units?
  double desiredPos; // cm, 0 is full retract
  double desiredVel;

  PIDController posPID = new PIDController(4.0, 0.0015, 0.125);
  PIDFController hwVelPID = new PIDFController(0.02, 0.0, 0, 0.0285);
  final NeoServo servo = new NeoServo(CAN.CLIMBER, posPID, hwVelPID, false); //check invert

  public Climber() {
    hwVelPID.copyTo(servo.getController().getPIDController(), 0);
    servo.setConversionFactor(conversionFactor) // in cm
        .setSmartCurrentLimit(STALL_CURRENT, FREE_CURRENT)
        .setVelocityHW_PID(maxVel, maxAccel)
        .setTolerance(posTol, velTol)
        .setMaxVelocity(maxVel)
        .burnFlash();
    servo.setPosition(StartPosition);
  }

  /**
   * Extend arm to position given.
   *
   * @param pos Desired position in cm from fully retracted position
   */
  public void setSetpoint(double pos) {
    desiredPos = pos;
    servo.setSetpoint(pos);
  }

  // lines 46-56 are for testing only
  /**
   * Testing command, sets the arms to a comanded velocity
   *
   * @param vel Sets arms to a specific velocity (in cm/sec)
   */
  public void setArmVelocity(double vel) {
    desiredVel = vel;
    servo.setVelocityCmd(vel);
  }

  public void setClimberPos(double pos){
    servo.setPosition(pos);
  }

  public double getClimberPos() {
    return servo.getPosition();
  }

  public double getClimberVelocity() {
    return servo.getVelocity();
  }

  public boolean atSetpoint() {
    return servo.atSetpoint();
  }

  public void ClampVel(double vel) {
    MathUtil.clamp(vel, maxVel, -maxVel);
  }

  public void ClampAccel(double accel) {
    MathUtil.clamp(accel, maxAccel, -maxAccel);
  }
    public Command getWatcher() {
    return new ClimberWatcherCmd();
  }

  // TODO: Calibration helpers
  // void SetZero()

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    servo.periodic();
  }

  public double getCurrent() {
    return servo.getController().getOutputCurrent();
  }

   /*
   * Watcher commmand puts network table data for intake.
   * 
   */
  class ClimberWatcherCmd extends WatcherCmd {
    NetworkTableEntry nt_desiredVel;
    NetworkTableEntry nt_desiredPos;
    NetworkTableEntry nt_currentVel;
    NetworkTableEntry nt_currentPos;
    NetworkTableEntry nt_atSetpoint;

    @Override
    public String getTableName() {
      return Climber.this.getName();
    }

    public void ntcreate() {
      NetworkTable table = getTable();
      nt_desiredVel = table.getEntry("desiredSpeed");
      nt_desiredPos = table.getEntry("desiredPos");
      nt_currentVel = table.getEntry("currentSpeed");
      nt_currentPos = table.getEntry("currentPos");
      nt_atSetpoint = table.getEntry("atSetpoint");

    }

    public void ntupdate() {
      nt_desiredVel.setDouble(desiredVel);
      nt_desiredPos.setDouble(desiredPos);
      nt_currentVel.setDouble(getClimberVelocity());
      nt_currentPos.setDouble(getClimberPos());
      nt_atSetpoint.setBoolean(atSetpoint());
    }
  } // watcher command

}
