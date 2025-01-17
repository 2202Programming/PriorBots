/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot2019.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot2019.Robot;
import frc.robot2019.commands.arm.ArmStatePositioner;
import frc.robot2019.subsystems.ArmSubsystem.Position;

public class GripperPositionCommand extends Command {
  double timeout;
  double height;
  double projx;
  double error;
  
  // Phyical values from sub-systems as needed
  Position armPosition;
  private ArmStatePositioner armPositioner;

  public GripperPositionCommand(double height, double projx, double error, double timeout) {
      this.height = height;
      this.projx = projx;
      this.timeout = timeout;
      this.error = Math.abs(error);
  }

  @Override
  public void initialize() {
      setTimeout(timeout);
      // Assume that the ArmStatePositioner is the only type of default command used
      armPositioner = Robot.arm.getArmPositioner();;
      armPositioner.setPosition(height, projx); // sets the CommandManagers h/x output
      armPosition = Robot.arm.getArmPosition();
  }

  @Override
  public void execute() {
    armPosition = Robot.arm.getArmPosition();
    //Robot.m_cmdMgr.cmdPosition(height, projx); // once should be fine
  }

  @Override
  public boolean isFinished() {
      double h_err = Math.abs(armPosition.height - height);
      double x_err = Math.abs(armPosition.projection - projx);
      boolean posGood = (h_err < error) && (x_err < error);
      return posGood || isTimedOut();
  }
}