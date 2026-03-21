/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.timbot.subsystem;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.timbot.Constants.PCM;

public class Trigger extends SubsystemBase {

  final DoubleSolenoid trigger;

  public Trigger() {
    trigger = new DoubleSolenoid(2, PneumaticsModuleType.REVPH, PCM.TRIGGER_BACK, PCM.TRIGGER_FORWARD);
    trigger.set(DoubleSolenoid.Value.kReverse);
  }

  @Override
  public void periodic() {
   
  }

  //Trigger API
  public Command trigger_fire(){
    return runOnce(() -> {
        trigger.set(DoubleSolenoid.Value.kForward);
    });
  }
  public Command trigger_reset(){
    return runOnce(() -> {
        trigger.set(DoubleSolenoid.Value.kReverse);
    });
  }

}
