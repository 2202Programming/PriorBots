// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2024.commands.Shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2024.subsystems.ShooterServo;

/*
 * Calibrates the shooter by movint towards the lowest angle (zero extension)
 * and stopping when the servo stops moving.
 */
public class CalibrateZero extends Command {
    final ShooterServo shooterServo;
    final double CalVelocity = -1.0;
    final double VelTol = 0.25; //[cm/s]
    final int DelayCount = 5;
    
    private int count;
    private boolean finished = false;

  //Lower the shooter servo until it stops, then set relative encoder to zero.
  public CalibrateZero() {
    shooterServo = RobotContainer.getSubsystem(ShooterServo.class);
    addRequirements(shooterServo);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterServo.setExtensionVelocity(CalVelocity);
    finished = false;
    count = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((++count > DelayCount) && (Math.abs(shooterServo.getExtensionVelocity()) <  VelTol)) {
      finished = true;
    }
    SmartDashboard.putNumber("ShooterServo/Velocity", shooterServo.getExtensionVelocity());
    SmartDashboard.putNumber("ShooterServo/Position", shooterServo.getAngle());
    SmartDashboard.putNumber("ShooterServo/Current", shooterServo.getCurrent());
    System.out.println("Servo zeroing: velocity="+shooterServo.getExtensionVelocity()
      +" Postion=" + shooterServo.getAngle()
      +" Current=" + shooterServo.getCurrent());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterServo.setExtensionVelocity(0.0);
    shooterServo.setExtensionPosition(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
