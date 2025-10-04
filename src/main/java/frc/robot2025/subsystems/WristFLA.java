// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot2025.Constants.AnalogIn;
import frc.robot2025.Constants.CAN;

public class WristFLA extends SubsystemBase {

  private static final double TOLERANCE = 0.04;
  // values for voltage to position measurement on analogInput
  private static final double KV = 1.94123305;
  private static double KV_offset = .255126927;

  private final SparkMax motor = new SparkMax(CAN.WRIST, MotorType.kBrushed);
  private final AnalogInput vPositionSensor = new AnalogInput(AnalogIn.Wrist);
  private final BangBangController bandBangController = new BangBangController(TOLERANCE);

  private double distance = 0.0; // measured
  private double distanceCmd = 0.0; // commanded

  public static final double PICKUP_POSITION = 1.8; // pickup position from source
  public static final double MID_POSITION = 1.5; // drop position for L2/3
  public static final double Q3_POSITION = 0.3; // drop position for L2/3
  public static final double ALGAE_REMOVAL_POSITION = 1.0;
  public static final double DROP_POSITION = 0.0; // drop position for L2/3

  public WristFLA() {
    SmartDashboard.putData(bandBangController);
    // use coast mode so we can "force" the server by hand if needed when disabled
    SparkBaseConfig driveCfg = new SparkMaxConfig().inverted(false).idleMode(IdleMode.kCoast);
    motor.configure(driveCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // this setPosition() won't happen until robot is enabled
    setPosition(2.5);    
  }

  public void setPosition(double position) {
    distanceCmd = position;
    bandBangController.setSetpoint(position);
  }

  @Override
  public void periodic() {
    distance = getDistance();
    double output;

    if (!atSetpoint()) {
      if (distanceCmd >= distance) {
        output = bandBangController.calculate(distance, distanceCmd);
        motor.set(output);
      } else {
        output = bandBangController.calculate(distanceCmd, distance);
        motor.set(-output);
      }
    } else {
      motor.set(0);
    }
  }

  public double getDistance(){
    return (vPositionSensor.getVoltage() - KV_offset) * KV;
  }

  //stop shouldn't be needed, should always drive to distanceCmd
  public void stop() {
    motor.set(0);
  }

  public boolean atSetpoint() {
    return bandBangController.atSetpoint();
  }

}
