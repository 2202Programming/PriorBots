/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.timbot.subsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Elevation extends SubsystemBase {
  /**
   * Creates a new Elevation.
   */

  public CANSparkMax actuator = new CANSparkMax(18, MotorType.kBrushless);
  public AnalogInput volts = new AnalogInput(0);
  public PIDController linearControlPID = new PIDController(getPosition(), getPosition(), getPosition());
  public double height = 0.0;
  public double speed = 0.0; //comanded power between 1.0 and -1.0

  public Elevation(CANSparkMax actuator) {
    volts.setAverageBits(4);
  }

  @Override
  public void periodic() {
    System.out.println(this.getPosition());
    this.getHeight();
  }

  public void setActuatorAngle(double angle) {

  }

  public double getHeight() {
    height = 11*2.54 + this.getPosition();
    return height;
  }

  public double getPosition() { //in inches
    System.out.println(volts.getVoltage());
    return volts.getVoltage() * 1.2 * 2.54; //basiclly 0-5 volts * 6 inches/5 * 2.54 cm/in
  }

  public void setPosition(double height) {
      this.height = height;
  }

  public void setSpeed(double speed) {
    this.speed = speed;
  }

}
