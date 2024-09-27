// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.timbot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.timbot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class Flywheel extends SubsystemBase {
  /** Creates a new Flywheel. */
  private TalonFX frontMotor;
  private TalonFX backMotor;

  public Flywheel(TalonFX frontMotor, TalonFX backMotor) {
    this.backMotor = backMotor;
    this.frontMotor = frontMotor;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void spinSpeed(double speed) {
    frontMotor.set(ControlMode.Velocity, speed);
    backMotor.set(ControlMode.Velocity, speed);
  }

  public void spinPower(double power) {
    frontMotor.set(ControlMode.PercentOutput, power);
    backMotor.set(ControlMode.PercentOutput, power);
  }

  public boolean upToSpeed() {
    if ((frontMotor.getSensorCollection().getIntegratedSensorVelocity() >= Constants.SHOOTER.MIN_SHOOTER_SPEED)
        && (backMotor.getSensorCollection().getIntegratedSensorVelocity() >= Constants.SHOOTER.MIN_SHOOTER_SPEED)) {
      return true;
    }
    return false;
  }

}
