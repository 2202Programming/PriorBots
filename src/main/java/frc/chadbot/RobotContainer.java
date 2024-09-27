// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.chadbot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.chadbot.Constants.DriverPrefs;
import frc.chadbot.Constants.Shooter;
import frc.chadbot.commands.IntakeCommand;
import frc.chadbot.commands.IntakeCommand.IntakeMode;
import frc.chadbot.commands.MagazineGatedCommand;
import frc.chadbot.commands.MoveIntake;
import frc.chadbot.commands.MoveIntake.DeployMode;
import frc.chadbot.commands.MovePositioner;
import frc.chadbot.commands.MovePositioner.PositionerMode;
import frc.chadbot.commands.ResetPosition;
import frc.chadbot.commands.Shoot.VelShootGatedCommand;
import frc.chadbot.commands.auto.auto_cmd;
import frc.chadbot.commands.swerve.DriveController;
import frc.chadbot.commands.swerve.DriveControllerDrivetrain;
import frc.chadbot.commands.swerve.LimelightDriveCmd;
import frc.chadbot.commands.test.ClimberTestRotRate;
import frc.chadbot.subsystems.Intake_Subsystem;
import frc.chadbot.subsystems.Limelight_Subsystem;
import frc.chadbot.subsystems.Magazine_Subsystem;
import frc.chadbot.subsystems.Positioner_Subsystem;
import frc.chadbot.subsystems.Sensors_Subsystem;
import frc.chadbot.subsystems.SwerveDrivetrain;
import frc.chadbot.subsystems.climber.Climber;
import frc.chadbot.subsystems.hid.HID_Xbox_Subsystem;
import frc.chadbot.subsystems.hid.SideboardController.SBButton;
import frc.chadbot.subsystems.hid.XboxAxis;
import frc.chadbot.subsystems.hid.XboxButton;
import frc.chadbot.subsystems.hid.XboxPOV;
import frc.chadbot.subsystems.ifx.DriverControls.Id;
import frc.chadbot.subsystems.shooter.Shooter_Subsystem;
import frc.robot.ux.Dashboard;

public class RobotContainer {
  static RobotContainer rc;

  public static RobotContainer RC() {
    return rc;
  }

  public final Dashboard dashboard;
  public Shooter_Subsystem shooter = null;
  public final HID_Xbox_Subsystem driverControls;
  public Sensors_Subsystem sensors = null;
  public Intake_Subsystem intake = null;
  public SwerveDrivetrain drivetrain = null;
  public Magazine_Subsystem magazine = null;
  public Climber climber = null;
  public final Limelight_Subsystem limelight;
  public final Positioner_Subsystem positioner;

  public static String auto_path_name = "NONE";

  public DriveController m_driveController = null;
  public DriveControllerDrivetrain m_dDriveControllerDrivetrain = null;
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
    limelight = new Limelight_Subsystem();
    positioner = new Positioner_Subsystem();
    driverControls = new HID_Xbox_Subsystem(DriverPrefs.VelExpo, DriverPrefs.RotationExpo, DriverPrefs.StickDeadzone);

    // These are hardware specific
    if (Constants.HAS_DRIVETRAIN)
      drivetrain = new SwerveDrivetrain();
    if (Constants.HAS_SHOOTER)
      shooter = new Shooter_Subsystem();
    if (Constants.HAS_MAGAZINE)
      magazine = new Magazine_Subsystem();
    if (Constants.HAS_INTAKE) {
      intake = new Intake_Subsystem();
    
    }
    if (Constants.HAS_CLIMBER)
      climber = new Climber();

    if (Constants.HAS_DRIVETRAIN && Constants.HAS_SHOOTER && Constants.HAS_MAGAZINE) {
       // set default commands
      mag_default_cmd = new MagazineGatedCommand(1.0);
      magazine.setDefaultCommand(mag_default_cmd);
      // swd = new DriveCmd(drivetrain, driverControls);
      // swd = new LimelightDriveCmd(drivetrain, driverControls, limelight);
      m_driveController = new DriveController(mag_default_cmd);
      // drivetrain.setDefaultCommand(m_driveController);
    }

    if(!Constants.IS_COMPETITION_BOT){
      m_dDriveControllerDrivetrain = new DriveControllerDrivetrain();
    }

    //TEST CODE  - Swingcheck wont return but puts values on Nettable
    driverControls.bind(Id.SwitchBoard, SBButton.Sw26).whileHeld(new SwingCheck(SwingCheck.Axis.Pitch, -40,-42, -1.0, 1.0).withTimeout(60.0));  

    // //setup the dashboard programatically, creates any choosers, screens
    // dashboard = new Dashboard(this);

    setDriverButtons();
    setAssistantButtons();
    
    // Sideboard 
    if (Constants.HAS_CLIMBER) { driverControls.bind(Id.SwitchBoard, SBButton.Sw21).whileHeld(new 
      // warning - PitAlign command use Driver's DPAD, RB and, LB. DPL-can we run this in TEST mode?
     PitAlignClimber(driverControls, Id.Driver, climber, 2.0, 5.0)); //[in/s] [deg/s]
      driverControls.bind(Id.SwitchBoard, SBButton.Sw22).whenPressed(new MidClimb(climber));
      driverControls.bind(Id.SwitchBoard, SBButton.Sw23).whenPressed(new TraverseClimb(climber));
      driverControls.bind(Id.SwitchBoard, SBButton.Sw24).whileHeld(new SequentialCommandGroup(
        new MoveArmsTo(climber, "To Angle 0", (climber.getLeftExtInches() + climber.getRightExtInches())/2, 0, true, true),
        new MoveArmsTo(climber, "To zero", 0, 0, true, true)));
      //driverControls.bind(Id.SwitchBoard, SBButton.Sw25).whileHeld(new ClimberTestRotRate(climber, 20, -30, 65)); //use pit-zero to start
      driverControls.bind(Id.SwitchBoard, SBButton.Sw25).whileHeld(
        new ClimberTestRotRate(climber, 40.0, -70.0, 30.0));
      
            //new ClimberTestVelocity(climber, 4, 0.0, 12)); //use pit-zero to start
            //new ClimberTestRotRate(climber, 15, -30, 60));
            //new ClimberTestRotRatePercent(climber, 0.5, -20, 40));
            //new ClimberTestRotOscillation(climber));
    }
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
    if (Constants.HAS_DRIVETRAIN && Constants.IS_COMPETITION_BOT) {
      driverControls.bind(Id.Driver, XboxButton.B).whenPressed(m_driveController::cycleDriveMode);
      driverControls.bind(Id.Driver, XboxButton.Y).whenPressed(new InstantCommand(() -> { drivetrain.resetAnglePose(Rotation2d.fromDegrees(-180)); })); //-180 reset if intake faces drivers
      driverControls.bind(Id.Driver, XboxAxis.TRIGGER_LEFT).whenPressed(m_driveController::setRobotCentric);
      driverControls.bind(Id.Driver, XboxAxis.TRIGGER_LEFT).whenReleased(m_driveController::setFieldCentric);   
      driverControls.bind(Id.Driver, XboxAxis.TRIGGER_RIGHT).whenPressed(m_driveController::turnOnShootingMode);
      driverControls.bind(Id.Driver, XboxAxis.TRIGGER_RIGHT).whenReleased(m_driveController::turnOffShootingMode);
    }
    if (Constants.HAS_DRIVETRAIN && !Constants.IS_COMPETITION_BOT) {
      driverControls.bind(Id.Driver, XboxButton.B).whenPressed(m_dDriveControllerDrivetrain::cycleDriveMode);
      driverControls.bind(Id.Driver, XboxButton.Y).whenPressed(new InstantCommand(() -> { drivetrain.resetAnglePose(Rotation2d.fromDegrees(-180)); })); //-180 reset if intake faces drivers
      driverControls.bind(Id.Driver, XboxAxis.TRIGGER_LEFT).whenPressed(m_dDriveControllerDrivetrain::setRobotCentric);
      driverControls.bind(Id.Driver, XboxAxis.TRIGGER_LEFT).whenReleased(m_dDriveControllerDrivetrain::setFieldCentric);   
      driverControls.bind(Id.Driver, XboxAxis.TRIGGER_RIGHT).whenPressed(m_dDriveControllerDrivetrain::turnOnShootingMode);
      driverControls.bind(Id.Driver, XboxAxis.TRIGGER_RIGHT).whenReleased(m_dDriveControllerDrivetrain::turnOffShootingMode);
    }


    // RB limelight toggle
    driverControls.bind(Id.Driver, XboxButton.X).whenPressed(new InstantCommand(limelight::toggleLED));

    //temporary for navx/pigeon testing
    driverControls.bind(Id.Driver, XboxPOV.POV_UP).whenPressed(new InstantCommand(()->{ sensors.disableNavx(true); }));
    driverControls.bind(Id.Driver, XboxPOV.POV_DOWN).whenPressed(new InstantCommand(()->{ sensors.disableNavx(false); }));

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
    driverControls.bind(Id.SwitchBoard, SBButton.Sw13).whenActive(new ResetPosition(Autonomous.startPose3));

    if (Constants.HAS_INTAKE) {
      driverControls.bind(Id.Assistant, XboxButton.LB).whenPressed(new MoveIntake(DeployMode.Toggle));
      //vertical intake controls - manual control of intake and side rollers,not the magazine
      driverControls.bind(Id.Assistant, XboxButton.A).whileHeld(new IntakeCommand((() -> 0.6), () -> 0.5, IntakeMode.LoadCargo));
      driverControls.bind(Id.Assistant, XboxButton.B).whileHeld(new IntakeCommand((() -> 0.35), () -> 0.5, IntakeMode.ExpellCargo));
    }

    if (Constants.HAS_MAGAZINE && Constants.HAS_SHOOTER) {
      // Positioner binds :)
      driverControls.bind(Id.Assistant, XboxButton.RB).whenPressed(new MovePositioner(PositionerMode.Toggle));

      // Magazine Commands with intake sides, and intake roller
      driverControls.bind(Id.Assistant, XboxButton.X).whileHeld(mag_default_cmd.getFeedCmd());
      driverControls.bind(Id.Assistant, XboxButton.Y).whileHeld(mag_default_cmd.getEjectCmd());

      driverControls.bind(Id.Assistant, XboxAxis.TRIGGER_RIGHT).whileHeld(new VelShootGatedCommand(Shooter.DefaultSettings,     mag_default_cmd));
      driverControls.bind(Id.Assistant, XboxPOV.POV_LEFT)      .whileHeld(new VelShootGatedCommand(Shooter.shortVelocity,       mag_default_cmd));
      driverControls.bind(Id.Assistant, XboxPOV.POV_UP)        .whileHeld(new VelShootGatedCommand(Shooter.shortMediumVelocity, mag_default_cmd));
      driverControls.bind(Id.Assistant, XboxPOV.POV_DOWN)      .whileHeld(new VelShootGatedCommand(Shooter.mediumVelocity,      mag_default_cmd));
      driverControls.bind(Id.Assistant, XboxPOV.POV_RIGHT)     .whileHeld(new VelShootGatedCommand(Shooter.longVelocity,        mag_default_cmd));
    }
  }

  public Command getAutonomousCommand() {
    return new auto_cmd();
  }

}
