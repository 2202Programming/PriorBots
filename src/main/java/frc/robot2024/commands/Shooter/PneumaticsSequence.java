// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2024.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2024.subsystems.Intake;
import frc.robot2024.subsystems.Shooter;

public class PneumaticsSequence extends Command {
  /** Creates a new PneumaticsTest. */
  final Shooter shooter;
  final Intake intake;
  
  public PneumaticsSequence() {
    this.shooter = RobotContainer.getSubsystem(Shooter.class);
    this.intake = RobotContainer.getSubsystem(Intake.class);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setMaxVelocity(60.0);
    intake.setAngleSetpoint(25.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(intake.angleAtSetpoint()){
      shooter.deploy();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
