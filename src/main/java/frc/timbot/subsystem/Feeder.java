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

public class Feeder extends SubsystemBase {

  final DoubleSolenoid feeder;

  public Feeder() {
    feeder = new DoubleSolenoid(2, PneumaticsModuleType.REVPH, PCM.FEEDER_BACK, PCM.FEEDER_FORWARD);
    feeder.set(DoubleSolenoid.Value.kReverse);
  }

  @Override
  public void periodic() {
   
  }

  //Feeder API
  public Command fire(){
    return runOnce(() -> {
        feeder.set(DoubleSolenoid.Value.kForward);
    });
  }
  public Command reset(){
    return runOnce(() -> {
        feeder.set(DoubleSolenoid.Value.kReverse);
    });
  }

}
