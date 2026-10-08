// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2026.command;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib2202.builder.RobotContainer;
import frc.robot2026.subsystems.Hopper;
import frc.robot2026.subsystems.Intake;

public class AgitateV2 extends SequentialCommandGroup {

  public AgitateV2() {
    this(false);
  }

  public AgitateV2(boolean hopperForward) {
    Intake intake = RobotContainer.getSubsystem("intake");
    Hopper hopper = RobotContainer.getSubsystem(Hopper.class);

    setName("AgitateV2");
    addRequirements(intake, hopper);

    double hopper_fwd = 0.65; //[%pwr]
    // optionally, keep hopperbelt moving forward for shooting
    double hopper_rev = hopperForward ? hopper_fwd : -0.65; //[%pwr]
    
    addCommands(   // New agitate command (will run hopper forward and back fast to unscramble the fuel and then run belts into indexer for 2 seconds)
        hopper.cmdBeltPct(hopper_fwd),
        intake.cmdPctPwr(0.70),
        new WaitCommand(0.1),
        hopper.cmdBeltPct(hopper_rev),
        intake.cmdPctPwr(-0.70),
        new WaitCommand(0.25),
        hopper.cmdBeltPct(hopper_fwd),
        intake.cmdPctPwr(-0.70),
        new WaitCommand(0.25),
        hopper.cmdBeltPct(hopper_rev),
        intake.cmdPctPwr(0.70),
        new WaitCommand(0.25),
        hopper.cmdBeltPct(hopper_rev),
        intake.cmdPctPwr(0.70),
        new WaitCommand(0.25),
        hopper.cmdBeltPct(hopper_fwd),
        intake.cmdPctPwr(0.65),
        new WaitCommand(2.5)
    );  
  }
}
