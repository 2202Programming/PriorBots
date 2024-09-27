/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Stick;

public class Elevation extends SubsystemBase {
  /**
   * Creates a new Elevation.
   */

  public TalonSRX actuator;

  public Elevation(TalonSRX actuator) {
    this.actuator = actuator;

    // https://github.dev/CrossTheRoadElec/Phoenix-Examples-Languages

    // Initialization procedure brought from
    // https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java%20Talon%20FX%20(Falcon%20500)/PositionClosedLoop/src/main/java/frc/robot/Robot.java
    this.actuator.configFactoryDefault();
    this.actuator.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition,
        Constants.Elevation.kPIDLoopIdx, Constants.Elevation.kTimeoutMs);

    this.actuator.setSensorPhase(Constants.Elevation.kSensorPhase);
    this.actuator.setInverted(Constants.Elevation.kMotorInvert);

    this.actuator.configNominalOutputForward(0, Constants.Elevation.kTimeoutMs);
    this.actuator.configNominalOutputReverse(0, Constants.Elevation.kTimeoutMs);
    this.actuator.configPeakOutputForward(1, Constants.Elevation.kTimeoutMs);
    this.actuator.configPeakOutputReverse(-1, Constants.Elevation.kTimeoutMs);

    /**
     * Config the allowable closed-loop error, Closed-Loop output will be neutral
     * within this range. See Table in Section 17.2.1 for native units per rotation.
     */
    this.actuator.configAllowableClosedloopError(0, Constants.Elevation.kPIDLoopIdx, Constants.Elevation.kTimeoutMs);

    /* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
    this.actuator.config_kF(Constants.Elevation.kPIDLoopIdx, Constants.Elevation.kGains.kF,
        Constants.Elevation.kTimeoutMs);
    this.actuator.config_kP(Constants.Elevation.kPIDLoopIdx, Constants.Elevation.kGains.kP,
        Constants.Elevation.kTimeoutMs);
    this.actuator.config_kI(Constants.Elevation.kPIDLoopIdx, Constants.Elevation.kGains.kI,
        Constants.Elevation.kTimeoutMs);
    this.actuator.config_kD(Constants.Elevation.kPIDLoopIdx, Constants.Elevation.kGains.kD,
        Constants.Elevation.kTimeoutMs);

    setActuatorAngle(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setActuatorAngle(double angle) {
    actuator.set(ControlMode.Position, angleToPosition(angle));
    Stick.log("Running setActuatorAngle angle= " + angle);
  }

  public double angleToPosition(double angle) {
    return angle;
  }

}
