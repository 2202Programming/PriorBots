// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2024.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2024.subsystems.Shooter;
import frc.robot2024.subsystems.ShooterServo;
import frc.robot2024.subsystems.Transfer;

public class TestShoot extends Command {
  /** Creates a new ShootTest. */
  Shooter shooter;
  double speed;
  Transfer transfer;
  public TestShoot(double speed) {
        this.shooter = RobotContainer.getSubsystemOrNull(Shooter.class);
        if(this.shooter == null){
          this.shooter = RobotContainer.getSubsystem(ShooterServo.class);
        }
        this.speed = speed;
        this.transfer = RobotContainer.getSubsystem(Transfer.class);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    transfer.setSpeed(20.0);
    shooter.setRPM(speed, speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setRPM(0, 0);
    }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
