/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.timbot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Trigger extends SubsystemBase {
  /**
   * Creates a new Elevation.
   */
  final DoubleSolenoid.Value FIREING = Value.kForward; // expand
  final DoubleSolenoid.Value IDLE = Value.kReverse; // retract

  private DoubleSolenoid solenoid;
  // private Compressor compressor;

  public Trigger(DoubleSolenoid solenoid) {
    this.solenoid = solenoid;
    idle();
    // compressor = new Compressor(1); // 1 is CAN ID of PCM
    solenoid.clearAllPCMStickyFaults();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // if (!compressor.enabled()) {
    // compressor.start();
    // }
  }

  public void fire() {
    solenoid.set(FIREING);
  }

  public void idle() {
    solenoid.set(IDLE);
  }

}