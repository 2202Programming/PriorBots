// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2024.commands.Shooter;

import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.lib2202.Constants;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.subsystem.BlinkyLights;
import frc.lib2202.subsystem.BlinkyLights.BlinkyLightUser;
import frc.robot2024.subsystems.ShooterServo;
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

public class ShooterServoSequence extends BlinkyLightUser {
  boolean stay; // if true, shooter stays in current position
  final ShooterServo shooter;
  final Transfer transfer;
  final double TransferSpeed = 40.0; // [cm/s]
  final double NoteTravelDist = 15.0; // [cm] max distance needed for note to move
  final int DONE_COUNT = (int) Math.ceil((NoteTravelDist / TransferSpeed) / Constants.DT);

  double speed;
  double rpm_tolerance = 0.02; // was 3%, .5% used for testing accuracy
  // TODO .5% worked but was slow speed it up
  double angle;
  int count = 0;
  Phase phase;
  boolean auto;
  final DistanceInterpretor interp;
  boolean useInterp = false;

  public enum Phase {
    WaitingForSetpoints, WaitingForFinish, Finished;
  }

  public ShooterServoSequence(double angle, double speed, boolean stay) {
    this(angle, speed, stay, false);
  }

  public ShooterServoSequence(double angle, double speed, boolean stay, boolean auto) {
    this.stay = stay;
    this.angle = angle;
    this.speed = speed;
    this.auto = auto;
    this.shooter = RobotContainer.getSubsystem(ShooterServo.class);
    this.transfer = RobotContainer.getSubsystem(Transfer.class);
    interp = DistanceInterpretor.getSingleton();
    addRequirements(shooter, transfer);
  }

  public ShooterServoSequence(double angle, double speed) {
    this(angle, speed, false);
  }

  /** With distance interpret(Automatically changing Angle) */
  public ShooterServoSequence() {
    this(0.0, 0.0, false); // alliance dependant so need to be in init
    useInterp = true;
  }

  public ShooterServoSequence(boolean auto) {
    this(0.0, 0.0, false, auto); // alliance dependant so need to be in init
    useInterp = true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (useInterp) {// Just if we are changing rpm automatically
      interp.setTarget(); // uses alliance and speaker tag
      angle = interp.getTargetAngle();
      speed = interp.getTargetRPM();
    }
    count = 0;
    phase = Phase.WaitingForSetpoints;

    // start motor right away
    shooter.setAngleSetpoint(angle);
    shooter.setRPM(speed + shooter.adjustment, speed - shooter.adjustment); // placeholder
    System.out.println("***ShooterSequence:init.... motors on ***");
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
      case WaitingForSetpoints:
        if (shooter.atSetpoint() && shooter.isAtRPM(speed * rpm_tolerance)) { // prev at 100RPM
          transfer.setSpeed(40.0);
          System.out.println("***ShooterSequence:Setpoints reached, transfer moving ....***");
          phase = Phase.WaitingForFinish;
        }
        break;
      case WaitingForFinish:
        if (++count >= DONE_COUNT) // || transfer.senseNote()) // end cmd faster if we see Note go by
        {
          phase = Phase.Finished;
          System.out.println("***ShooterSequence:finished....***");
        }
        break;
      default:
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    transfer.setHasNote(false);
    transfer.setSpeed(0.0);
    if (!auto) {
      shooter.setRPM(0.0, 0.0);
    }
    if (!stay && shooter.getAngle() > ShooterServo.MAX_SERVO_INTAKE_ANGLE) {
      shooter.setAngleSetpoint(ShooterServo.MAX_SERVO_INTAKE_ANGLE);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return phase == Phase.Finished;
  }
}
