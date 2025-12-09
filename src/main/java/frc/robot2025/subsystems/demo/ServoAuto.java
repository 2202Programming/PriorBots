// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.subsystems.demo;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib2202.builder.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ServoAuto extends SequentialCommandGroup {
  /** Creates a new ServoAuto. */
  public ServoAuto() {
    // Use addRequirements() here to declare subsystem dependencies.
    SimpleServo c = RobotContainer.getSubsystem("Servo0");
    addRequirements(c);
    Command Step1 = c.cmdPosition(0.9);
    Command Step2 = Commands.waitSeconds(1);
    Command Step3 = c.cmdPosition(0.6);
    Command Step4 = Commands.waitSeconds(1);
    Command Step5 = c.cmdPosition(0.3);
    Command Step6 = Commands.waitSeconds(1);
    Command Step7 = c.cmdPosition(0);
    addCommands(Step1, Step2, Step3, Step4, Step5,Step6,Step7);
  }
}
