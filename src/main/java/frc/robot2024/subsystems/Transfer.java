// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2024.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot2024.Constants.CAN;
import frc.robot2024.Constants.DigitalIO;
import frc.lib2202.command.WatcherCmd;
import frc.lib2202.util.PIDFController;

public class Transfer extends SubsystemBase {

  // constants for geometry of transfer
  final static double radius = 1.27 * 2.0 * Math.PI; // 1.27 radius in cm --> probably circumference lol
  final static double gearRatio = 1.0 / 35.0; // 35 motor turns -> 1 roller shaft turn [verified gr]
  final static double conversionFactor = radius * gearRatio; // [cm/rotations]

  static final double MIN_SPEED = -1.0, MAX_SPEED = 1.0; // example looks like Pct Pwr
  public static final double MAX_VEL = 42.0;

  // calc Kff for vel control from measured (RPS / %pwr)
  final static double Kff = (1.0 / 43.2); // full pwr gave 43.2 [cm/s]
  final PIDFController transferPID = new PIDFController(0.015, 0.0, 0.0, Kff);

  DigitalInput lightgate = new DigitalInput(DigitalIO.Transfer_LightGate);
  CANSparkMax transferMtr;
  final SparkPIDController transferMtrPid;
  final RelativeEncoder transferMtrEncoder;

  // state vars
  boolean hasNote = false;
  double speed_cmd; // for monitoring
  boolean senseNote_prev;

  /** Creates a new Transfer. */
  public Transfer() {
    transferMtr = new CANSparkMax(CAN.TRANSFER_MOTOR, CANSparkMax.MotorType.kBrushless);
    transferMtr.clearFaults();
    transferMtr.restoreFactoryDefaults();
    transferMtr.setInverted(true);
    transferMtrPid = transferMtr.getPIDController();
    transferMtrEncoder = transferMtr.getEncoder();
    transferMtrEncoder.setPositionConversionFactor(conversionFactor);
    transferMtrEncoder.setVelocityConversionFactor(conversionFactor / 60.0); // min to sec
    transferPID.copyTo(transferMtrPid, 0);
    transferMtrPid.setOutputRange(MIN_SPEED, MAX_SPEED, 0);
    transferMtr.burnFlash();

    transferMtrEncoder.setPosition(0.0);
  }


  /*
   * true when note is blocking light gate
   */
  public boolean senseNote() {
    return !lightgate.get();
  }

  /*
   * true - note passed light gate and then cleared past gate
   * commands should stop motor on hasNote() == true.
   */
  public boolean hasNote() {
    return hasNote;
  }

  /*
   * sets if we have a note or not for powerup or initization in commands
   */
  public void setHasNote(boolean note_state) {
    hasNote = note_state;
    senseNote_prev = false;
  }

  /*
   * speed [cm/s]
   */
  public void setSpeed(double speed) {
    transferMtrPid.setReference(speed, ControlType.kVelocity, 0);
    this.speed_cmd = speed;
    // transferMtr.set(Transfer_Constants.TRANSFER_MOTOR_ON);
  }

  public double getTransferVelocity() {
    return transferMtrEncoder.getVelocity();
  }

  public Command getWatcher() {
    return new TransferWatcherCmd();
  }

  /*
   * watch gate during periodic so the hasNote() is accurate
   */
  @Override
  public void periodic() {
    if (!hasNote) {
      if (senseNote() && !senseNote_prev) {
        senseNote_prev = true;
        hasNote = true;
      // } else if (senseNote_prev) {
      //   // high to low edge seen, we have it. Cmd will use timer to position correctly
      //   // and stop the intakeMtr based on direction and timing.
      //   hasNote = true;
      // }
    }
  }
}

  class TransferWatcherCmd extends WatcherCmd {
    // NetworkTableEntry nt_lightgate;
    NetworkTableEntry nt_lightgate;
    NetworkTableEntry nt_Vel;
    NetworkTableEntry nt_velcmd;
    NetworkTableEntry nt_have_note;
    NetworkTableEntry nt_pos;

    @Override
    public String getTableName() {
      return Transfer.this.getName();
    }

    public void ntcreate() {
      NetworkTable table = getTable();
      nt_lightgate = table.getEntry("senseNote");
      nt_Vel = table.getEntry("velMeas");
      nt_velcmd = table.getEntry("velCmd");
      nt_have_note = table.getEntry("haveNote");
      nt_pos = table.getEntry("pos_");

      // default value for mutables
      // example nt_maxArbFF.setDouble(maxArbFF);
    }

    public void ntupdate() {
      nt_lightgate.setBoolean(senseNote());
      nt_Vel.setDouble(getTransferVelocity());
      nt_velcmd.setDouble(speed_cmd);
      nt_have_note.setBoolean(hasNote());

      nt_pos.setDouble(transferMtrEncoder.getPosition());

      // get mutable values
      // example maxArbFF = nt_maxArbFF.getDouble(maxArbFF);

    }
  } // watcher command
}
