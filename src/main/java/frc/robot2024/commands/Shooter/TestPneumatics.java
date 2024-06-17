// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2024.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2024.subsystems.Shooter;

public class TestPneumatics extends Command {
  /** Creates a new PneumaticsTest. */
  final Shooter shooter;
  boolean extend;

  public TestPneumatics(boolean extend) {
    this.extend = extend;
    this.shooter = RobotContainer.getSubsystem(Shooter.class);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (extend) {
      shooter.deploy();
      System.out.println("deploying");
    } else {
      shooter.retract();
      System.out.println("retracting");
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
