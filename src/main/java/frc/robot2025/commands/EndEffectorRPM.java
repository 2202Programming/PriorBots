// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2025.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2025.subsystems.EndEffector_Subsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class EndEffectorRPM extends Command {
  /** Creates a new ClimberSetPos. */
  final EndEffector_Subsystem endEffector;
  double RPM;
  String label;
  public EndEffectorRPM(double RPM) {
    this(RPM, "default button");
  }
  public EndEffectorRPM(double RPM, String label) {
    endEffector = RobotContainer.getSubsystem("endEffectorSubsystem");
    this.label=label;
    this.RPM=RPM;
    SmartDashboard.putNumber("End Effector RPM "+ label, RPM);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(endEffector.pieceReady()) {
      endEffector.setRPM(SmartDashboard.getNumber("End Effector RPM "+ label, RPM));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    endEffector.setRPM(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // Mr. L wants to know when this cmd should end, do we really want it to run forever???

    return false;
  }
}
