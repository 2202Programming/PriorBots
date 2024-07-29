// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2024.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot2024.Constants.PWM;

public class AmpMechanism extends SubsystemBase {
  final double GearRatio = 40.0/60.0;
  final double turns = 5.0;
  public final double parked = 0.0;
  public final double extended = 0.118;
  public final double field_goal = 0.08;
  public double desiredPos;
   Servo left_servo = new Servo(PWM.LEFT_AMP_MECHANISM);
   Servo right_servo = new Servo(PWM.RIGHT_AMP_MECHANISM);
  /** Creates a new AmpMechanism. */
  public AmpMechanism() {
    SmartDashboard.putNumber("AMP MECHANISM DEBUG", 0.5);
    setServo(parked);
  }
  public void setServo(double cmdPos){
    left_servo.set(1.0-cmdPos);
    right_servo.set(cmdPos);
  }

  @Override
  public void periodic() {
    desiredPos = SmartDashboard.getNumber("AMP MECHANISM DEBUG", 0.5);
    // This method will be called once per scheduler run
  }
}
