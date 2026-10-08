// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2026.subsystems;

import static frc.lib2202.Constants.DEGperRAD;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANrange;
import frc.lib2202.command.WatcherCmd;
import frc.robot2026.Constants.CAN;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RangeSensor extends SubsystemBase {
  /** Creates a new Sensors. */

  private final CANrange canRange_L;
  private final CANrange canRange_R;
  private final double separation_Distance;
  private double canRange_L_Distance;
  private double canRange_R_Distance;
  private double wall_angle;
 

  // Specify the CAN bus name, e.g., "rio" (roboRIO), "can0" (Linux), or a
  // CANivore name
  final CANBus kCANrangeCANbus = new CANBus("rio");

  public RangeSensor() {
    this(.5);  // estimated default distance between Range sensors
  }

  public RangeSensor(double sep_dist_m) {
    setName("range");
    separation_Distance = sep_dist_m;
    // Construct the CANrange object
    canRange_L = new CANrange(CAN.CANRANGE_L_CAN, kCANrangeCANbus);
    canRange_R = new CANrange(CAN.CANRANGE_R_CAN, kCANrangeCANbus);
    canRange_L_Distance = 0.0;
    canRange_R_Distance = 0.0;
    getWatcherCmd();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    canRange_L_Distance = canRange_L.getDistance().getValueAsDouble();
    canRange_R_Distance = canRange_R.getDistance().getValueAsDouble();
    wall_angle = angle();
  }

  double angle(){
    double diff = canRange_L_Distance - canRange_R_Distance;
    return Math.atan2(diff, separation_Distance);
  }

  // Add a watcher so we can see stuff on network tables
  public WatcherCmd getWatcherCmd() {
    return this.new RangeSensorWatcher();
  }

  public double getWallAngle() {
    return wall_angle;
  }

  public double getWallAngleDeg() {
    return wall_angle*DEGperRAD;
  }

  public double getCANRangeL() {
    return canRange_L_Distance;
  }

  public double getCANRangeR() {
    return canRange_R_Distance;
  }

  class RangeSensorWatcher extends WatcherCmd {
    RangeSensorWatcher() {
        addEntry("Range Left", RangeSensor.this::getCANRangeL);
        addEntry("Range Right", RangeSensor.this::getCANRangeR);  
        addEntry("Range Angle", RangeSensor.this::getWallAngleDeg); 
      }
  }
}
