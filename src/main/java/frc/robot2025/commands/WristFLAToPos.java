// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2025.subsystems.WristFLA;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class WristFLAToPos extends Command {
  WristFLA servo = RobotContainer.getSubsystem(WristFLA.class);
  double pos;
  String label;

  public WristFLAToPos(double pos) {
    this(pos, "");
  }
  public WristFLAToPos(double pos, String label) {
    this.pos = pos;
    this.label = label;
    SmartDashboard.putNumber("WristFLAToPos button "+ label, pos);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //servo.setPos(pos);
    System.out.println("position:" + SmartDashboard.getNumber("WristFLAToPos button "+ label, pos));
    servo.setPosition(SmartDashboard.getNumber("WristFLAToPos button "+ label, pos));
  }

  @Override
  public void execute() {
    servo.setPosition(SmartDashboard.getNumber("WristFLAToPos button "+ label, pos));
  }

  @Override
  public void end(boolean interrupted) {
    servo.stop();
  }

  @Override
  public boolean isFinished() {
    return servo.atSetpoint(); //change depending on acceptable error
  }
}
