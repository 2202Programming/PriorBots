// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2024.commands.Shooter;

import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.subsystem.BlinkyLights;
import frc.lib2202.subsystem.BlinkyLights.BlinkyLightUser;
import frc.robot2024.subsystems.Intake;
import frc.robot2024.subsystems.Shooter;
import frc.robot2024.subsystems.Transfer;

/**
 * Button pressed starts command
 * Make sure we detect a note
 * Turn the shooter motors on
 * Once our shooter motors reach the proper speed, turn on the transfer motors
 * Once the lightgate no longer detects the note, wait x amount of frames
 * After x amount of frames, turn the blinky lights red, then turn everything
 * off (command ended)
 */

public class ShooterSequenceForAuto extends BlinkyLightUser {
  /** Creates a new ShooterSequence. */
  //use simple Shooter, even if ShooterServo is created because this command can work with either.
  final Shooter shooter;
  final Transfer transfer;
  final Intake intake;
  final int DONE_COUNT = 20;
  final int PNEUMATICS_DONE_COUNT = 5;
  double speed;
  int pneumatics_count = 0;
  int count = 0;
  Phase phase;
  boolean shootHigh;

  public enum Phase {
    HasNote, ShooterMotorOn, TransferMotorOn, Finished;
  }

  public ShooterSequenceForAuto(boolean shootHigh, double speed) {
    this.shootHigh = shootHigh;
    this.speed = speed;
    this.shooter = RobotContainer.getSubsystem(Shooter.class);
    this.transfer = RobotContainer.getSubsystem(Transfer.class);
    this.intake = RobotContainer.getSubsystem(Intake.class);
    addRequirements(shooter, transfer, intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public ShooterSequenceForAuto(double speed){
    this(false, speed);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("***ShooterSequence:init....***");
    count = 0;
    pneumatics_count = 0;
    phase = Phase.HasNote;
    if(shootHigh){
      intake.setMaxVelocity(60.0);
      intake.setAngleSetpoint(91);
    }
  }

  public Color8Bit colorProvider() {
    // make sure not is safely in our possession before going back
    return (!transfer.hasNote()) ? BlinkyLights.RED : BlinkyLights.GREEN;
  };

  @Override
  public boolean requestBlink() {
    return false; // we want solid lights
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (phase) {
      case HasNote:
      System.out.println("***ShooterSequence:HasNote....***");
          if(shootHigh){
            shooter.deploy();
            pneumatics_count++;
          }
          shooter.setRPM(speed, speed); // placeholder
          if(pneumatics_count >= PNEUMATICS_DONE_COUNT || !shootHigh){
          phase = Phase.ShooterMotorOn;
          }
        break;
      case ShooterMotorOn:
        System.out.println("***ShooterSequence:ShooterMotorOn....***");
        if (shooter.isAtRPM(100)) {
          transfer.setSpeed(35.0);
          phase = Phase.TransferMotorOn;
          System.out.println("***ShooterSequence:TransferMotorOn....***");
        }
        break;
      case TransferMotorOn:  
        count++;
        if (count >= DONE_COUNT) {
          phase = Phase.Finished;
          System.out.println("***ShooterSequence:finished....***");
        }
        break;
      case Finished:
        
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    transfer.setHasNote(false);
    transfer.setSpeed(0.0);
    shooter.setRPM(0.0, 0.0);
    shooter.retract();
    intake.setMaxVelocity(60.0);  //[deg/s] 2.sec to retract
    intake.setAngleSetpoint(91.0); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return phase == Phase.Finished;
  }
}
