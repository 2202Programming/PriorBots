// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.subsystems.demo;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib2202.builder.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ServoAuto extends SequentialCommandGroup {
  /** Creates a new ServoAuto. */
  public ServoAuto() {
    // Use addRequirements() here to declare subsystem dependencies.
    SimpleServo servo = RobotContainer.getSubsystem("Servo0");
    this.addCommands(servo.cmdPositionWaitForModel(0),
    servo.cmdPositionWaitForModel(1.0),
    servo.cmdPositionWaitForModel(0.5),
    servo.cmdPositionWaitForModel(0.25),
    servo.cmdPositionWaitForModel(0.0));
  }}