// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2023.commands.Intake.Washer;


import frc.base.RobotContainerOrig;
import edu.wpi.first.wpilibj2.command.Command;
import frc.base.Constants.Intake_Constants;
import frc.robot2023.subsystems.Intake;


public class reverseIntake extends Command {
  /** Creates a new activateIntake. */
  final Intake intake;

  public reverseIntake() {
    intake =  RobotContainerOrig.RC().intake;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setIntakeSpeed(-Intake_Constants.IntakeMotorStrength);
    intake.carwashReverse();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.intakeOff();
    intake.carwashOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //Idea: when the button is released edit a variable to return false
    //We may also want to end it when 2/3 color sensors are triggered
    return true;
  }
}