// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.chadbot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.chadbot.Constants.CAN;
import frc.chadbot.Constants.DigitalIO;

public class Magazine_Subsystem extends SubsystemBase {
  /** Creates a new Magazine. */
  private TalonSRX top_wheel;
  

  /*Definitions*/
  final DigitalInput lightGateLower = new DigitalInput(DigitalIO.MAGAZINE_GATE1);
  final DigitalInput lightGateUpper = new DigitalInput(DigitalIO.MAGAZINE_GATE2);

  private int logCounter = 0;

  //Constructor
  public Magazine_Subsystem() {
    // copy the PID settings to the hardware
    top_wheel = new TalonSRX(CAN.MAG_TOP_WHEEL);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    log();
  }

  //sets the belts to a speed

  //Drivewheel on applies a percent output between -1 and 1
  public void driveWheelOn(double speed){
    top_wheel.set(TalonSRXControlMode.PercentOutput, speed);
  }
  //defaultDriveWheelOn sets percent output to 1
  public void defaultDriveWheelOn(){
    driveWheelOn(1);
  }
  //defaultDriveWheelOn sets percent output to 0
  public void driveWheelOff(){
    driveWheelOn(0);
  }

  //reverses direction of rotation to expell cargo
  public void expellCargo(double speed){
    driveWheelOn(-speed);
  }
  
  //lets us know if cargo is detected
  public boolean lowerGateBlocked() {
    return  !lightGateLower.get();
  }
  public boolean upperGateBlocked(){
    return !lightGateUpper.get();
  }

  public boolean bothGatesBlocked() {
    return upperGateBlocked() && lowerGateBlocked();
  }

  private void log(){
    logCounter++;
    if (logCounter%10 == 0) {
      SmartDashboard.putBoolean("Ball in Upper", upperGateBlocked());
      SmartDashboard.putBoolean("Ball in Lower", lowerGateBlocked());
    }

  }

}


