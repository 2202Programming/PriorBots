// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot2025.Constants;
import frc.lib2202.command.WatcherCmd;
import frc.lib2202.util.NeoServo;
import frc.lib2202.util.PIDFController;

public class Climber extends SubsystemBase {
  NeoServo servo;
  boolean disable_servo = false;

  PIDFController hwVelocity_PID = new PIDFController(0.04, 0.00000500, 0.0, 5.0 / 180.0 / 1.2); // [deg/s]
  PIDController  swPosition_PID = new PIDController(3.0 ,0, 0);  //[deg]

  //convert to deg/s units at the geared output
  final double GearRatio = 9.0 * 5.0 * 5.0 * 4.0; // sprocket gear is 64/16
  final double conversionFactor = 360.0 / GearRatio;  // [deg/rot]

  // Motor settings for Servo
  final int STALL_CURRENT = 100;
  final int FREE_CURRENT = 20;
  final boolean motor_inverted = true;
  // Servo speed/positions
  final double maxVel = 100.0;  // [winch deg/s]
  final double maxAccel = 75.0; // [winch deg/s/s]
  final double posTol =  1.0; // tol = tolerance [deg]
  final double velTol =  0.5; // [deg/s]
  final double PowerUpPosition = 0.0; //[deg]

  double cmdPos;
  double cmdVel;

  final SparkBase controller;
  final SparkClosedLoopController cl_controller;
  ClimberWatcherCmd watcher;

  /** Creates a new Climber. */
  public Climber() {
    // setup any other hardware Pid values, like Izone 
    hwVelocity_PID.setIZone(200.0); //[deg/s]  outside this region ignore integral

    servo = new NeoServo(Constants.CAN.CLIMBER, swPosition_PID, hwVelocity_PID, motor_inverted);
    servo.setConversionFactor(conversionFactor)  // units should be [deg] and [deg/s]
        .setSmartCurrentLimit(STALL_CURRENT, FREE_CURRENT)
        .setVelocityHW_PID(maxVel, maxAccel)
        .setTolerance(posTol, velTol)
        .setMaxVelocity(maxVel);
        this.watcher = new ClimberWatcherCmd();

        // get the controllers out of the server so we can monitor in our watcher.
        controller = servo.getController();
        cl_controller = controller.getClosedLoopController();

        servo.setSmartCurrentLimit(STALL_CURRENT, FREE_CURRENT);

        // We are going to use position command, so need to set power on POS
        // this also means some PIT trim needs to be done when shutting off for a match
        zero();
  }

  public void zero() {
    servo.setVelocityCmd(0.0);
    servo.setPosition(PowerUpPosition);   
  }

  // velocity control only used for testing, normal cmds will use position
  public void setVelocity(double vel) {
    cmdVel = vel;
    servo.setVelocityCmd(vel);
  }

  public double getVelocity() {
    return servo.getVelocity();
  }

  public boolean atVelocity() {
    return Math.abs(getVelocity() - cmdVel) < velTol;
  }

  public double getPosition() {
    return servo.getPosition();
  }

  public double getSetpoint() {
    return servo.getSetpoint();
  }

  public void setSetpoint(double pos) {
    servo.setSetpoint(pos);
  }

  public double getMaxVel() {
    return servo.getMaxVel();
  }

  public void setMaxVelocity(double vel) {
    servo.setMaxVelocity(vel);
  }

  public double getCmdVelocity() {
    return cmdVel;
  }

  public boolean atSetpoint(){
    return servo.atSetpoint();
  }

  public void setIAccum(double iaccum) {
    cl_controller.setIAccum(iaccum);
  }

  public void setDutyCycleMode(double duty) {
    // this is a hack to force the servo motor into power mode
    // Return to normal servo mode with duty==0.0
    disable_servo = !(duty == 0.0);
    controller.set(duty);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // power mode testing, disable servo if testing with duty-cycle
    if (!disable_servo) {
      servo.periodic();
    }
  }

  class ClimberWatcherCmd extends WatcherCmd {
    NetworkTableEntry nt_cmdVelocity;  
    NetworkTableEntry nt_measVelocity;
    NetworkTableEntry nt_measPosition;
    NetworkTableEntry nt_cmdPosition; // setpoint
    NetworkTableEntry nt_atSetpoint;
    NetworkTableEntry nt_current;
    NetworkTableEntry nt_appliedOutput;
    NetworkTableEntry nt_Iacc;
    NetworkTableEntry nt_mtrTemp;

    @Override
    public String getTableName() {
      return "Climber";
    }

    public void ntcreate() {
      NetworkTable table = getTable();
      nt_cmdVelocity = table.getEntry("cmdVelocity");
      nt_measVelocity = table.getEntry("measVelocity");
      nt_measPosition = table.getEntry("measPosition");
      nt_cmdPosition = table.getEntry("cmdPosition");
      nt_atSetpoint = table.getEntry("atSetpoint");
      nt_current = table.getEntry("mtrCurrent");
      nt_appliedOutput = table.getEntry("appliedOutput");
      nt_Iacc = table.getEntry("Iaccum");
      nt_mtrTemp = table.getEntry("mtrTemp");
    }

    public void ntupdate() {
      nt_cmdVelocity.setDouble(getCmdVelocity());
      nt_measVelocity.setDouble(getVelocity());
      nt_measPosition.setDouble(getPosition());
      nt_cmdPosition.setDouble(getSetpoint());
      nt_atSetpoint.setBoolean(atSetpoint());
      nt_current.setDouble(controller.getOutputCurrent());
      nt_appliedOutput.setDouble(controller.getAppliedOutput()*100.0);
      nt_Iacc.setDouble(cl_controller.getIAccum());
      nt_mtrTemp.setDouble(controller.getMotorTemperature());

    }
  }
}
