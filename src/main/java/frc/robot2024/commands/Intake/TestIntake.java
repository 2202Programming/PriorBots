// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2024.commands.Intake;

import frc.lib2202.subsystem.BlinkyLights.BlinkyLightUser;
import frc.lib2202.builder.RobotContainer;
import frc.robot2024.subsystems.Intake;


public class TestIntake extends BlinkyLightUser {

    /** Creates a new intakeForward. */
    public final Intake intake;
    double speed;
    public TestIntake(double speed) {
        this.speed = speed;
        this.intake = RobotContainer.getSubsystem(Intake.class);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        intake.setIntakeSpeed(speed);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intake.setIntakeSpeed(0.0);
    }

    @Override
    public boolean isFinished() {
        // use limit switches to stop if we go too far
        return false;
    }
}
