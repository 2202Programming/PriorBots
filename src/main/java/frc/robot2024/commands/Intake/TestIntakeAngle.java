// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2024.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2024.subsystems.Intake;

public class TestIntakeAngle extends  Command {
    //Safe speed for moving to limit switch

    /** Creates a new intakeForward. */
    public final Intake intake;
    double angleVelocity; //[deg/s]
    public TestIntakeAngle(double angleVelocity) {
        this.intake = RobotContainer.getSubsystem(Intake.class);
        this.angleVelocity = angleVelocity;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("*******SPEED******" + angleVelocity);
        intake.setAngleVelocity(angleVelocity);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) { 

        // shouldn't be needed on new bot
        // intake.setMaxVelocity(1.0); // test worked, turns off vel-cmd at end so bounce isn't chased.
        intake.setAngleVelocity(0.0);             
    }

    // Returns true when the command should end, we end when count hits DONE_COUNT
    @Override
    public boolean isFinished() {
        return false;
        
    }
}
