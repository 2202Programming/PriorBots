// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.chadbot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.chadbot.Constants.Autonomous;
import frc.chadbot.Constants.DriverPrefs;
import frc.chadbot.Constants.Shooter;
import frc.chadbot.commands.IntakeCommand;
import frc.chadbot.commands.MagazineGatedCommand;
import frc.chadbot.commands.MoveIntake;
import frc.chadbot.commands.MovePositioner;
import frc.chadbot.commands.ResetPosition;
import frc.chadbot.commands.IntakeCommand.IntakeMode;
import frc.chadbot.commands.MoveIntake.DeployMode;
import frc.chadbot.commands.MovePositioner.PositionerMode;
import frc.chadbot.commands.Shoot.VelShootGatedCommand;
import frc.chadbot.commands.auto.auto_cmd;
import frc.chadbot.commands.swerve.DriveControllerDrivetrain;
import frc.chadbot.commands.swerve.DriveControllerWithShooter;
import frc.chadbot.commands.swerve.LimelightDriveCmd;
import frc.chadbot.subsystems.Intake_Subsystem;
import frc.lib2202.subsystem.Limelight;
import frc.chadbot.subsystems.Magazine_Subsystem;
import frc.chadbot.subsystems.Positioner_Subsystem;
import frc.chadbot.subsystems.Sensors_Subsystem;
import frc.lib2202.subsystem.swerve.SwerveDrivetrain;
import frc.lib2202.subsystem.hid.HID_Xbox_Subsystem;
import frc.lib2202.subsystem.hid.SwitchboardController.SBButton;
import frc.chadbot.subsystems.ifx.DriverControls.Id;
import frc.chadbot.subsystems.shooter.Shooter_Subsystem;
import frc.chadbot.RobotSpec_ChadBot;
//import frc.chadbot.RobotSpec_ChadBot.RobotNames;
import frc.chadbot.ux.Dashboard;

public class RobotContainer {
  static RobotContainer rc;

  public static RobotContainer RC() {
    return rc;
  }

  // controllers
  CommandXboxController driverController = new CommandXboxController(0);
  CommandXboxController opController = new CommandXboxController(1);

  // TODO WHERE TF IS SENSORS SUBSYSTEM AT 
  public final Dashboard dashboard;
  public Shooter_Subsystem shooter = null;
  public final HID_Xbox_Subsystem driverControls;
  public Sensors_Subsystem sensors = null;
  public Intake_Subsystem intake = null;
  public SwerveDrivetrain drivetrain = null;
  public Magazine_Subsystem magazine = null;
//  public Climber climber = null;
  public Limelight limelight = null;
  public Positioner_Subsystem positioner = null;

  public static String auto_path_name = "NONE";

  public DriveControllerWithShooter m_driveController = null;
  public DriveControllerDrivetrain m_driveControllerDrivetrain = null;
  public Command drivetrainCommand = null;
  public RobotSpec_ChadBot m_robotSpecs;
  MagazineGatedCommand mag_default_cmd;

  // modifiable commands
  // DriveCmd swd;
  LimelightDriveCmd swd;



  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    RobotContainer.rc = this;

    // these can get created on any hardware setup
    sensors = new Sensors_Subsystem();
    dashboard = new Dashboard(rc);
    driverControls = new HID_Xbox_Subsystem(DriverPrefs.VelExpo, DriverPrefs.RotationExpo, DriverPrefs.StickDeadzone);
    m_robotSpecs = new RobotSpec_ChadBot(System.getenv("serialnum"));

    // These are hardware specific
    if (m_robotSpecs.getSubsystemConfig().HAS_DRIVETRAIN)
      drivetrain = new SwerveDrivetrain();
    if (m_robotSpecs.getSubsystemConfig().HAS_SHOOTER)
      shooter = new Shooter_Subsystem();
    if (m_robotSpecs.getSubsystemConfig().HAS_MAGAZINE)
      magazine = new Magazine_Subsystem();
    if (m_robotSpecs.getSubsystemConfig().HAS_POSITIONER)
      positioner = new Positioner_Subsystem();
    if (m_robotSpecs.getSubsystemConfig().HAS_INTAKE)
      intake = new Intake_Subsystem();
//    if (m_robotSpecs.getSubsystemConfig().HAS_CLIMBER)
//      climber = new Climber();
    if (m_robotSpecs.getSubsystemConfig().HAS_LIMELIGHT)
      limelight = new Limelight();

    if (m_robotSpecs.getSubsystemConfig().HAS_DRIVETRAIN && m_robotSpecs.getSubsystemConfig().HAS_SHOOTER && m_robotSpecs.getSubsystemConfig().HAS_MAGAZINE) {
       // set default commands
      mag_default_cmd = new MagazineGatedCommand(1.0);
      magazine.setDefaultCommand(mag_default_cmd);
      // swd = new DriveCmd(drivetrain, driverControls);
      // swd = new LimelightDriveCmd(drivetrain, driverControls, limelight);
      m_driveController = new DriveControllerWithShooter(mag_default_cmd);
      // drivetrain.setDefaultCommand(m_driveController);
      drivetrainCommand = m_driveController;
    }

    else if(!m_robotSpecs.getSubsystemConfig().IS_COMPETITION_BOT){ //set up driveController version for swervebot
      m_driveControllerDrivetrain = new DriveControllerDrivetrain();
      drivetrainCommand = m_driveControllerDrivetrain;
    }

    //TEST CODE  - Swingcheck wont return but puts values on Nettable
    //driverControls.bind(Id.SwitchBoard, SBButton.Sw26).whileHeld(new SwingCheck(SwingCheck.Axis.Pitch, -40,-42, -1.0, 1.0).withTimeout(60.0));  

    // //setup the dashboard programatically, creates any choosers, screens
    // dashboard = new Dashboard(this);

    setDriverButtons();

    setAssistantButtons();
     
      // Sideboard 
//      if (m_robotSpecs.getSubsystemConfig().HAS_CLIMBER) { driverControls.bind(Id.SwitchBoard, SBButton.Sw21).whileTrue(new 
//        // warning - PitAlign command use Driver's DPAD, RB and, LB. DPL-can we run this in TEST mode?
//      PitAlignClimber(driverControls, Id.Driver, climber, 2.0, 5.0)); //[in/s] [deg/s]
//        driverControls.bind(Id.SwitchBoard, SBButton.Sw22).onTrue(new MidClimb(climber));
//        driverControls.bind(Id.SwitchBoard, SBButton.Sw23).onTrue(new TraverseClimb(climber));
//        driverControls.bind(Id.SwitchBoard, SBButton.Sw24).whileTrue(new SequentialCommandGroup(
//          new MoveArmsTo(climber, "To Angle 0", (climber.getLeftExtInches() + climber.getRightExtInches())/2, 0, true, true),
//          new MoveArmsTo(climber, "To zero", 0, 0, true, true)));
//        //driverControls.bind(Id.SwitchBoard, SBButton.Sw25).whileHeld(new ClimberTestRotRate(climber, 20, -30, 65)); //use pit-zero to start
//        driverControls.bind(Id.SwitchBoard, SBButton.Sw25).whileTrue(
//          new ClimberTestRotRate(climber, 40.0, -70.0, 30.0));
//        
//              //new ClimberTestVelocity(climber, 4, 0.0, 12)); //use pit-zero to start
//              //new ClimberTestRotRate(climber, 15, -30, 60));
//              //new ClimberTestRotRatePercent(climber, 0.5, -20, 40));
//              //new ClimberTestRotOscillation(climber));
//    }
  }

  /**
   * Driver xbox controller button bindings
   * <ul>
   * <li>B - Toggle drive mode</li>
   * <li>A - Trajectory Follow Test</li>
   * <li>Y - Reset Pose to Zero</li>
   * <li>X - Follow path off chooser</li>
   * </ul>
   */
  void setDriverButtons() {
    // B - Toggle drive mode
    if (m_robotSpecs.getSubsystemConfig().HAS_DRIVETRAIN && m_robotSpecs.getSubsystemConfig().IS_COMPETITION_BOT) {
      driverController.b().onTrue(new InstantCommand(() -> {m_driveController.cycleDriveMode();}));
      driverController.y().onTrue(new InstantCommand(() -> { drivetrain.resetAnglePose(Rotation2d.fromDegrees(-180)); })); //-180 reset if intake faces drivers
      driverController.leftTrigger().onTrue(new InstantCommand(() -> {m_driveController.setRobotCentric();}));
      driverController.leftTrigger().onFalse(new InstantCommand(() -> {m_driveController.setFieldCentric();}));   

      //dpl testing hack shooter
      driverController.rightTrigger().whileTrue(new VelShootGatedCommand(Shooter.shortVelocity,       mag_default_cmd));

      //driverController.rightTrigger().onTrue(new InstantCommand(() -> {m_driveController.turnOnShootingMode();}));
      //driverController.rightTrigger().onFalse(new InstantCommand(() -> {m_driveController.turnOffShootingMode();}));
    }
    if (m_robotSpecs.getSubsystemConfig().HAS_DRIVETRAIN && !m_robotSpecs.getSubsystemConfig().IS_COMPETITION_BOT) {
      driverControls.bind(Id.Driver, XboxButton.B).onTrue(new InstantCommand(() -> 
            { m_driveControllerDrivetrain.cycleDriveMode(); } ));
      driverControls.bind(Id.Driver, XboxButton.Y).onTrue(new InstantCommand(() -> 
            { drivetrain.resetAnglePose(Rotation2d.fromDegrees(-180)); })); //-180 reset if intake faces drivers
      driverControls.bind(Id.Driver, XboxAxis.TRIGGER_LEFT).onTrue(new InstantCommand(() -> 
            {m_driveControllerDrivetrain.setRobotCentric();} ));
      driverControls.bind(Id.Driver, XboxAxis.TRIGGER_LEFT).onFalse(new InstantCommand(() -> 
            {m_driveControllerDrivetrain.setFieldCentric();} ));   
      driverControls.bind(Id.Driver, XboxAxis.TRIGGER_RIGHT).onTrue(new InstantCommand(() -> 
            {m_driveControllerDrivetrain.turnOnShootingMode();} ));
      driverControls.bind(Id.Driver, XboxAxis.TRIGGER_RIGHT).onFalse(new InstantCommand(() -> 
            {m_driveControllerDrivetrain.turnOffShootingMode();} ));
    }


    // RB limelight toggle
    if (m_robotSpecs.getSubsystemConfig().HAS_LIMELIGHT)
      driverController.x().onTrue(new InstantCommand(limelight::toggleLED));

    //temporary for navx/pigeon testing
    // DPL - Binding pov broken for new lib model.  Mr.L wil fix.

    //driverControls.bind(Id.Driver, XboxPOV.POV_UP).onTrue(new InstantCommand(()->{ sensors.disableNavx(true); }));
    //driverControls.bind(Id.Driver, XboxPOV.POV_DOWN).onTrue(new InstantCommand(()->{ sensors.disableNavx(false); }));

  }

  // /**
  // * Assistant xbox controller button bindings
  // * <ul>
  // * <li> TBD </li>
  // * </ul>
  // */
  void setAssistantButtons() {
    // LB - toggle intake deploy
    // B - spin intake while held (to intake the ball)
    // A - spin intake while held (in reverse to expell the ball)
    // RT - spin shooter and index while held
    if (driverControls.isConnected(Id.SwitchBoard)) {
      driverControls.bind(Id.SwitchBoard, SBButton.Sw13).onTrue(new ResetPosition(Autonomous.startPose3));
    }
    
    if (m_robotSpecs.getSubsystemConfig().HAS_INTAKE) {
      opController.leftBumper().onTrue(new MoveIntake(DeployMode.Toggle));
      //driverControls.bind(Id.Assistant, XboxButton.LB).onTrue(new MoveIntake(DeployMode.Toggle));
      //vertical intake controls - manual control of intake and side rollers,not the magazine
      opController.a().whileTrue(new IntakeCommand((() -> 0.6), () -> 0.5, IntakeMode.LoadCargo));
      opController.b().whileTrue(new IntakeCommand((() -> 0.35), () -> 0.5, IntakeMode.ExpellCargo));
    }

    if (m_robotSpecs.getSubsystemConfig().HAS_MAGAZINE && m_robotSpecs.getSubsystemConfig().HAS_SHOOTER) {
      // Positioner binds :)
      opController.rightBumper().onTrue(new MovePositioner(PositionerMode.Toggle));

      // Magazine Commands with intake sides, and intake roller
      opController.x().whileTrue(mag_default_cmd.getFeedCmd());
      opController.y().whileTrue(mag_default_cmd.getEjectCmd());

      opController.rightTrigger().whileTrue(new VelShootGatedCommand(Shooter.DefaultSettings,     mag_default_cmd));
      opController.povLeft().whileTrue(new VelShootGatedCommand(Shooter.shortVelocity,       mag_default_cmd));
      opController.povUp().whileTrue(new VelShootGatedCommand(Shooter.shortMediumVelocity, mag_default_cmd));
      opController.povDown().whileTrue(new VelShootGatedCommand(Shooter.mediumVelocity,      mag_default_cmd));
      opController.povRight().whileTrue(new VelShootGatedCommand(Shooter.longVelocity,        mag_default_cmd));
    }
  }

  public Command getAutonomousCommand() {
    return new auto_cmd();
  }

}
