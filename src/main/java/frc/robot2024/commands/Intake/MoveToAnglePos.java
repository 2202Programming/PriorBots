// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2024.commands.Intake;

import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.base.RobotContainerOrig;
import frc.robot2023.subsystems.BlinkyLights;
import frc.robot2023.subsystems.Intake;
import frc.robot2023.subsystems.BlinkyLights.BlinkyLightUser;

public class MoveToAnglePos extends BlinkyLightUser {
  /** Creates a new AnglePos. */
  final Intake intake;
  double posCmd;
  double velLimit;
  
  public MoveToAnglePos(double posCmd, double velLimit) {
    this.intake = RobotContainerOrig.getSubsystem(Intake.class);
    this.posCmd = posCmd;
    this.velLimit = velLimit;
    addRequirements(intake);
  }

      /*
     * Control the blinkylights based on being at calibration point or not.
     */
    @Override
    public Color8Bit colorProvider() {
        // green when moving down, red moving back
        return (intake.getAngleSpeed() >0.0) ? BlinkyLights.GREEN : BlinkyLights.RED;
    };

    @Override
    public boolean requestBlink() {
        return false; // we want solid lights
    }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("STARTED MOVETO" + posCmd + ".............." + this.toString());
    intake.setMaxVelocity(velLimit);
    intake.setAngleSetpoint(posCmd);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.angleAtSetpoint();
  }
}
