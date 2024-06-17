// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2024.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2024.subsystems.ShooterServo;

/*
 * Moves shooter up outside the calibration switch then slowly brings down
 * to the calibration point.  Then sets the calibrated position.
 * 
 * After calibration, it moves the shooter to its firstShot position
 * at which point it can be turned off.
 * 
 * Note: this requires the powerup extension point to be set to the 
 * first shot position. See line 61 of the subsystem.
 *    EXTENSION_POWERUP_POS = Math.sin(FirstShot) * Hypotenuse - MIN_POSITION; // [cm]
 * 
 */
public class CalibrateWithLS extends Command {
    // Safe speed for moving to limit switch

    /** Creates a new intakeForward. */
    public final ShooterServo shooter;
    int count;
    final int DELAY_COUNT = 15;

    public enum Phase {
        Start, BeyondLS, Finished
    }

    Phase phase;

    public CalibrateWithLS() {
        this.shooter = RobotContainer.getSubsystem(ShooterServo.class);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        count = 0;
        phase = Phase.Start;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        switch (phase) {
            case Start:
                if (shooter.atLowLimit()) {
                    shooter.setExtensionVelocity(6.0);
                } else {
                    if (++count >= DELAY_COUNT) {
                        phase = Phase.BeyondLS;
                    }
                }
                break;
            case BeyondLS:
                shooter.setExtensionVelocity(-0.5);
                if (shooter.atLowLimit()) {
                    shooter.setExtensionVelocity(0.0);
                    shooter.setExtensionPosition(ShooterServo.SERVO_CALIB_EXT);
                    phase = Phase.Finished;
                }
                break;
            case Finished:
                break;
        }
    }
    @Override
    public void end(boolean interrupted) {
        // Now move shooter to NEW PowerUp angle, FirstShot
        shooter.setAngleSetpoint(shooter.FirstShot); //Turn off when finished moving
    }
    
    // Returns true when the command should end, we end when count hits DONE_COUNT
    @Override
    public boolean isFinished() {
        return (phase == Phase.Finished);
        // for alpha
        // return (angleVelocity < 0.0) ? intake.atReverseLimitSwitch() :
        // intake.atForwardLimitSwitch();
    }
}
