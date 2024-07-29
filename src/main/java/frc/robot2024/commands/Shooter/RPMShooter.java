// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2024.commands.Shooter;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2024.subsystems.Shooter;

public class RPMShooter extends Command {
  private final Shooter m_shooter;

  private double currentLeftMotorRPM;
  private double currentRightMotorRPM;

  private RelativeEncoder shooterLeftEncoder;
  private RelativeEncoder shooterRightEncoder;

  private double requestedLeftShooterRPM;
  private double requestedRightShooterRPM;
  private double requestedBothShooterRPM;

  private double lastRequestedLeftShooterRPM;
  private double lastRequestedRightShooterRPM;

  private double requestedP = 0.0001;
  private double requestedI = 0.0;
  private double requestedD = 0.0;

  private double currentP = 0.0;
  private double currentI = 0.0;
  private double currentD = 0.0;

  public RPMShooter() {
    m_shooter = RobotContainer.getSubsystem(Shooter.class);
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    requestedLeftShooterRPM = 0.0;
    requestedRightShooterRPM = 0.0;

    SmartDashboard.putNumber("Requested Left Shooter RPM", 0.0);
    SmartDashboard.putNumber("Requested Right Shooter RPM", 0.0);

    SmartDashboard.putNumber("Requested P", requestedP);
    SmartDashboard.putNumber("Requested I", requestedI);
    SmartDashboard.putNumber("Requested D", requestedD);

    // currentP = m_shooter.getP();
    // currentI = m_shooter.getI();
    // currentD = m_shooter.getD();

    SmartDashboard.putNumber("Current P", currentP);
    SmartDashboard.putNumber("Current I", currentI);
    SmartDashboard.putNumber("Current D", currentD);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentLeftMotorRPM = shooterLeftEncoder.getVelocity();
    currentRightMotorRPM = shooterRightEncoder.getVelocity();

    lastRequestedLeftShooterRPM = requestedLeftShooterRPM;
    lastRequestedRightShooterRPM = requestedRightShooterRPM;

    SmartDashboard.putNumber("Current Shooter RPM", currentLeftMotorRPM); // should be 1.0
    SmartDashboard.putNumber("Current Left Motor RPM", currentLeftMotorRPM);
    SmartDashboard.putNumber("Current Right Motor RPM", currentRightMotorRPM);

    requestedLeftShooterRPM = SmartDashboard.getNumber("Requested Left Shooter RPM", 0.0);
    requestedRightShooterRPM = SmartDashboard.getNumber("Requested Right Shooter RPM", 0.0);
    requestedBothShooterRPM = SmartDashboard.getNumber("Requested Shooter RPM: ", 0.0);

    if ((lastRequestedLeftShooterRPM != requestedLeftShooterRPM) ||
        (lastRequestedRightShooterRPM != requestedRightShooterRPM)) {
      m_shooter.setRPM(requestedLeftShooterRPM, requestedRightShooterRPM);
    }
    if ((requestedBothShooterRPM > 0)) {
      if ((requestedBothShooterRPM != requestedLeftShooterRPM)
          && (requestedBothShooterRPM != requestedRightShooterRPM)) {
        m_shooter.setRPM(requestedBothShooterRPM, requestedBothShooterRPM);
      } else {
        m_shooter.setRPM(requestedLeftShooterRPM, requestedRightShooterRPM);
      }
    }

    requestedP = SmartDashboard.getNumber("Requested P", requestedP);
    requestedI = SmartDashboard.getNumber("Requested I", requestedI);
    requestedD = SmartDashboard.getNumber("Requested D", requestedD);

    // currentP = m_shooter.getP();
    // currentI = m_shooter.getI();
    // currentD = m_shooter.getD();

    SmartDashboard.putNumber("Current P", currentP);
    SmartDashboard.putNumber("Current I", currentI);
    SmartDashboard.putNumber("Current D", currentD);

    setPIDs();
  }

  private void setPIDs() {
    // if (currentP != requestedP)
    //   m_shooter.setP(requestedP);
    // if (currentI != requestedI)
    //   m_shooter.setI(requestedI);
    // if (currentD != requestedD)
    //   m_shooter.setD(requestedD);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setRPM(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}