// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.commands.DropSequenceBaseCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.subsystem.UX.TrimTables.Trim;
import frc.robot2025.subsystems.WristFLA;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class setWristPos extends Command {
  final WristFLA wrist;
  final double setpoint;
  // boolean drop;
  final Trim trim;
  /** Creates a new setWristPos. */
  // public setWristPos(boolean drop) {
  //   wrist = RobotContainer.getSubsystem(WristFLA.class);
  //   this.drop = drop;
  //   trim = new UXTrim("trim", 0.0);
  //   if(drop){
  //     setpoint = WristFLA.MID_POSITION;
  //   } else {
  //     setpoint = WristFLA.PICKUP_POSITION;
  //   }
  //   // Use addRequirements() here to declare subsystem dependencies.
  // }
  public setWristPos(double setpoint, String name) {
    trim = new Trim("Wrist" , name);
    wrist = RobotContainer.getSubsystem(WristFLA.class);
    this.setpoint = setpoint;
    addRequirements(wrist);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wrist.setPosition(trim.getValue(setpoint));
  }

  // @Override
  // public void end(boolean interrupted){
  //   wrist.stop();
  // }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return wrist.atSetpoint();
  }
}
