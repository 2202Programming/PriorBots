// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2024.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2024.subsystems.Intake;

public class AngleCalibration extends  Command {
    //Safe speed for moving to limit switch

    /** Creates a new intakeForward. */
    final int DELAY = 5;
    int delay_count;
    public final Intake intake;
    double angleVelocity; //[deg/s]
    int count;
    final int DONE_COUNT = 5;
    public AngleCalibration(double angleVelocity) {
        this.intake = RobotContainer.getSubsystem(Intake.class);
        this.angleVelocity = angleVelocity;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("*******SPEED******" + angleVelocity);
        intake.setAngleVelocity(angleVelocity);
        count = 0;
        delay_count = 0;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        delay_count++;
        if(delay_count >= DELAY){
        count = (Math.abs(intake.getAngleVelocity()) < 3.0) ? ++count : 0;
        }
    }
    

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) { 
        // intake.setMaxVelocity(1.0); // test worked, turns off vel-cmd at end so bounce isn't chased.
        intake.setAngleVelocity(0.0);             
        // if (interrupted) return;

        //made it here, then we completed calibration, so set our position
         System.out.println("Calibration at " + intake.getAnglePosition());
        intake.setAnglePosition( (angleVelocity < 0.0) ? 0.0 : intake.getAnglePosition());
        intake.setAngleSetpoint(intake.getAnglePosition());
    }

    // Returns true when the command should end, we end when count hits DONE_COUNT
    @Override
    public boolean isFinished() {
        return count >=DONE_COUNT;
        //for alpha
        // return  (angleVelocity < 0.0) ? intake.atReverseLimitSwitch() : intake.atForwardLimitSwitch();      
        
    }
}
