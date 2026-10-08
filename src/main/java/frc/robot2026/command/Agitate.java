// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2026.command;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib2202.builder.RobotContainer;
import frc.robot2026.subsystems.Hopper;
import frc.robot2026.subsystems.Intake;

public class Agitate extends SequentialCommandGroup {

  public Agitate() {
    this(false);
  }

  public Agitate(boolean hopperForward) {
    Intake intake = RobotContainer.getSubsystem("intake");
    Hopper hopper = RobotContainer.getSubsystem(Hopper.class);

    setName("Agitate");
    addRequirements(intake, hopper);

    double hopper_fwd = 0.65; //[%pwr]
    // optionally, keep hopperbelt moving forward for shooting
    double hopper_rev = hopperForward ? hopper_fwd : -0.65; //[%pwr]
    
    addCommands(
        hopper.cmdBeltPct(hopper_fwd),
        intake.cmdPctPwr(0.65),
        new WaitCommand(0.5),
        hopper.cmdBeltPct(hopper_rev),
        intake.cmdPctPwr(-0.65),
        new WaitCommand(0.5),
        hopper.cmdBeltPct(hopper_fwd),
        intake.cmdPctPwr(-0.65),
        new WaitCommand(0.5),
        hopper.cmdBeltPct(hopper_rev),
        intake.cmdPctPwr(0.65),
        new WaitCommand(0.5)
    );  
  }
}
