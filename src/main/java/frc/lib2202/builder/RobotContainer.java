// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib2202.builder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.PDPMonitorCmd;
import frc.robot.commands.BlinkyLightColorCmd;
import frc.robot.commands.Shooter.ContinousAngleTracker;
import frc.robot.commands.Swerve.FieldCentricDrive;
import frc.robot2023.subsystems.Swerve.SwerveDrivetrain;
import frc.robot2023.subsystems.hid.HID_Xbox_Subsystem;
import frc.robot.util.RobotSpecs;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // enum for bindings add when needed
  public enum Bindings {
    Competition,
    DriveTest, Shooter_test, IntakeTesting, auto_shooter_test, new_bot_test, comp_not_comp, Etude
  }

  // Change the line below for testing, try not to commit a change 
  public static final frc.robot.RobotContainerOrig.Bindings bindings = Bindings.Competition;

  // The robot's subsystems and commands are defined here...
  static RobotContainer rc;
  final IRobotSpec robotSpecs;
  final HID_Xbox_Subsystem dc;
  final SwerveDrivetrain drivetrain;
  final SendableChooser<Command> autoChooser;

  // The following methods are unchecked, but the SystemConfig class does
  // check the types.
  // Use the string name when there are multiple instance of the subsystem
  @SuppressWarnings("unchecked")
  public static <T> T getSubsystem(String name) {
    return (T) rc.robotSpecs.mySubsystemConfig.getSubsystem(name);
  }

  @SuppressWarnings("unchecked")
  public static <T extends Subsystem> T getSubsystemOrNull(String name) {
    return (T) rc.robotSpecs.mySubsystemConfig.getObjectOrNull(name);
  }

  // Use this when there is only one instance of the Subsystem - preferred
  @SuppressWarnings("unchecked")
  public static <T extends Subsystem> T getSubsystem(Class<T> clz) {
    return (T) rc.robotSpecs.mySubsystemConfig.getSubsystem(clz);
  }

  // Use this when there is only one instance of the Subsystem and can deal with
  // nulls
  // in the context. It bypasses NPE checks. Know what you are doing.
  @SuppressWarnings("unchecked")
  public static <T extends Subsystem> T getSubsystemOrNull(Class<T> clz) {
    return (T) rc.robotSpecs.mySubsystemConfig.getObjectOrNull(clz.getSimpleName());
  }

  // Use this form when the RobotContainer object is NOT a Subsystem
  @SuppressWarnings("unchecked")
  public static <T> T getObject(String name) {
    return (T) rc.robotSpecs.mySubsystemConfig.getObject(name);
  }

  // Use this form when the RobotContainer object is NOT a Subsystem, and you can
  // deal with nulls
  @SuppressWarnings("unchecked")
  public static <T> T getObjectOrNull(String name) {
    return (T) rc.robotSpecs.mySubsystemConfig.getObjectOrNull(name);
  }

  public static boolean hasSubsystem(Class<? extends Subsystem> clz) {
    return rc.robotSpecs.mySubsystemConfig.hasSubsystem(clz);
  }

  public static IRobotSpec getRobotSpecs() {
    return rc.robotSpecs;
  }

  /**
   * The container for the robot.
   * 
   * You likely shouldn't need to edit this file.
   * For runtime config, see:   
   *  {@link Configs } - subsystems, watchers
   *  {@link Constants }
   *  {@link RegisteredCommands } - PathPlanner named commands
   *  {@link BindingsCompetition } - triggers
   * 
   *  public final Bindings - should normally point to Competition.
   * 
   */
  public RobotContainer() {
    RobotContainer.rc = this;
    robotSpecs = new IRobotSpec();
    robotSpecs.mySubsystemConfig.constructAll();
    autoChooser = RegisteredCommands.RegisterCommands();

    // Testing, but also to drive the drivers nuts...
    Command random_lights = new BlinkyLightColorCmd();
    random_lights.schedule();

    // Quiet some of the noise
    DriverStation.silenceJoystickConnectionWarning(true);

    // get subsystem vars as needed for bindings
    drivetrain = getSubsystem(SwerveDrivetrain.class);
    dc = getSubsystem("DC");

    /* Setup the commands below */
    if (drivetrain != null) {
      drivetrain.setDefaultCommand(new FieldCentricDrive());
    }

    new PDPMonitorCmd(); // auto scheduled, runs when disabled
    // uncomment to enable shooter angle tracking
    if(RobotContainer.getRobotSpecs().myRobotName.toString().equals("CompetitionBotBeta2024")){
    //TODO: DO WE NEED THIS?
    new ContinousAngleTracker(true);  //auto schedules
  }
    // make some noise if we are not on Competion bindings
    if (bindings != Bindings.Competition) {
      System.out.println(
          "*****************************************************************************\n" +
          "* Warning: Not using competition bindings, using: " + bindings.toString() + "\n" +
          "*****************************************************************************\n");
      BindingsOther.ConfigureOther(dc);
    } else {
      // Competition Bindings, see BindingsCompetition.java
      BindingsCompetition.ConfigueCompetition(dc);
    }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}
