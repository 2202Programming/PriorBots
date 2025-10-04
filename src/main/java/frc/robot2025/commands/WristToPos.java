// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib2202.builder.RobotContainer;
import frc.robot2025.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
//this version not used, see WristFLA
@Deprecated  
public class WristToPos extends InstantCommand {
  Wrist servo = RobotContainer.getSubsystem(Wrist.class);
  double pos;
  String label;

  public WristToPos(double pos) {
    this(pos, "");
  }
  public WristToPos(double pos, String label) {
    this.pos = pos;
    this.label = label;
    SmartDashboard.putNumber("WristToPos button "+ label, pos);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //servo.setPos(pos);
    servo.setPos(SmartDashboard.getNumber("WristToPos button "+ label, pos));
  }
}
