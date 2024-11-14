/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.timbot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Elevation extends SubsystemBase {
  /**
   * Creates a new Elevation.
   */

  public CANSparkMax actuator;
  public AnalogInput volts = new AnalogInput(0);
  public PIDController linearControlPID = new PIDController(getPosition(), getPosition(), getPosition());

  public Elevation(CANSparkMax actuator) {
    volts.setAverageBits(4);
  }

  @Override
  public void periodic() {
    System.out.println(this.getPosition());
  }

  public void setActuatorAngle(double angle) {

  }

  public double getHeight() {
    double height = 17.72 + this.getPosition();
    return height;
  }

  public double getPosition() { //in inches
    System.out.println(volts.getVoltage());
    return volts.getVoltage() * 1.2; //basiclly 0-5 volts * 6 inches/5
  }

}
