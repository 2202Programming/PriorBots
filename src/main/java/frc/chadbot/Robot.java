// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.chadbot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.chadbot.commands.Shoot.RPMShootCommandTune;



//////THIS WILL GET DELETED and incorporated into robotspec_chadbot



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private RobotContainer robotContainer;
  
  int disabledTimer = 0;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    LiveWindow.setEnabled(false);
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
    // todo() don't use statics
    robotContainer.limelight.disableLED();
    robotContainer.drivetrain.setCoastMode(); //make robot easier to push around to set up correctly
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    disabledTimer = 0;
    RobotContainer.RC().limelight.disableLED();
  }

  @Override
  public void disabledPeriodic() {
    disabledTimer++;
    if(disabledTimer == 200){ //wait a few seconds to stop momementum before releasing brakes
      robotContainer.drivetrain.setCoastMode(); //make robot easier to push around to set up correctly
      RobotContainer.RC().limelight.disableLED();
    }
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    robotContainer.drivetrain.setBrakeMode(); //brake mode for gameplay
    Command m_autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    RobotContainer.RC().limelight.enableLED();
    robotContainer.drivetrain.setBrakeMode(); //brake mode for gameplay
    CommandScheduler.getInstance().cancelAll();
    if(RobotContainer.RC().drivetrainCommand != null){
      CommandScheduler.getInstance().schedule(RobotContainer.RC().drivetrainCommand); //start driverController
      //CommandScheduler.getInstance().schedule(new RPMShootCommandTune());
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    LiveWindow.setEnabled(false);
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().schedule(new RPMShootCommandTune());
    CommandScheduler.getInstance().schedule(RobotContainer.RC().drivetrainCommand);
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    CommandScheduler.getInstance().run();
  }
}
