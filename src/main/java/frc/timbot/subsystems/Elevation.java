/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


package frc.timbot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;

import com.revrobotics.CANSparkMax;
import frc.timbot.Constants.CAN;

public class Elevation extends SubsystemBase {
  private static final double kp = 1.0;
  private static final double ki = 0;
  private static final double kd = 0;
  private static final double conversion = 6.0/5.0; //define later [in/volt]
  /** Creates a new LaunchAngle. */
  final CANSparkMax angleMotor = new CANSparkMax(CAN.ACTUATOR_SPARK, CANSparkMax.MotorType.kBrushed);
  final AnalogInput vPositionSensor = new AnalogInput(0); //[volts]
  final PIDController pid = new PIDController(kp, ki, kd); //change these names later
  private double distance = 0.0; //[cm]
  private double distanceCmd = 0.0; //[cm]
  

  public Elevation() {
  
  }

  @Override
  public void periodic() {
    distance = vPositionSensor.getVoltage()*conversion;
    double output = pid.calculate(distance, distanceCmd); //FOR FUTURE REFFERENCE GAVIN. THIS IS HOW TO PID.
    angleMotor.set(output);
    pid.setTolerance(5, 10); //What units?
  }

  public double getPos() {
    return distance;
  }

  public void setPoint(double point){ //0 is bottom, 6 is top
    if (point > 6) {
      point = 6;
    } else if (point < 0) {
      point = 0;
    }
    distanceCmd = point;
  }

  public boolean isAtPosition(){
    return pid.atSetpoint();
  }
}