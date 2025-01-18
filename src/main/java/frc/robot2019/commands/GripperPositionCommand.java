/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot2019.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2019.commands.arm.ArmStatePositioner;
import frc.robot2019.subsystems.ArmSubsystem;
import frc.robot2019.subsystems.ArmSubsystem.Position;

public class GripperPositionCommand extends Command {
  double timeout;
  double height;
  double projx;
  double error;
  
  // Phyical values from sub-systems as needed
  Position armPosition;
  final ArmSubsystem arm;
  final ArmStatePositioner armPositioner;
  final Timer timer;

  public GripperPositionCommand(double height, double projx, double error, double timeout) {
      arm = RobotContainer.getSubsystem(ArmSubsystem.class);
      armPositioner = arm.getArmPositioner();
      timer = new Timer();
      this.height = height;
      this.projx = projx;
      this.timeout = timeout;
      this.error = Math.abs(error);
  }

  @Override
  public void initialize() {
      timer.start();//setTimeout(timeout);
      // Assume that the ArmStatePositioner is the only type of default command used
      armPositioner.setPosition(height, projx); // sets the CommandManagers h/x output
      armPosition = arm.getArmPosition();
  }

  @Override
  public void execute() {
    armPosition = arm.getArmPosition();
    //Robot.m_cmdMgr.cmdPosition(height, projx); // once should be fine
  }

  @Override
  public boolean isFinished() {
      double h_err = Math.abs(armPosition.height - height);
      double x_err = Math.abs(armPosition.projection - projx);
      boolean posGood = (h_err < error) && (x_err < error);
      return posGood || timer.hasElapsed(timeout);
  }
}