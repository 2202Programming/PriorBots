/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot2019;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.subsystem.hid.ExpoShaper;
import frc.robot2019.commands.arm.ResetArmCommand;
import frc.robot2019.commands.cargo.AutoCargoIntakeCommand;
import frc.robot2019.commands.cargo.tests.IntakeTestCmd;
import frc.robot2019.commands.cargo.tests.OuttakeTestCmd;
// removed climber commands - no climber on bot
import frc.robot2019.commands.drive.CopilotControlCommand;
import frc.robot2019.commands.drive.InvertDriveControlsCommand;
import frc.robot2019.commands.drive.LimeLightArcadeDriveCommand;
import frc.robot2019.commands.drive.shift.AutomaticUpShiftCommand;
import frc.robot2019.commands.drive.shift.DownShiftCommand;
import frc.robot2019.commands.drive.shift.ToggleAutomaticGearShiftingCommand;
import frc.robot2019.commands.drive.shift.UpShiftCommand;
import frc.robot2019.commands.intake.VacuumCommand;
import frc.robot2019.input.XboxControllerButtonCode;
//import frc.robot2019.input.triggers.GeneralTrigger;
import frc.robot2019.input.triggers.JoystickTrigger;
import frc.robot2019.subsystems.ArmSubsystem;
import frc.robot2019.subsystems.ClimberSubsystem;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.onTrue(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileTrue(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.onFalse(new ExampleCommand());
  private XboxController driver = new XboxController(0);
  private XboxController assistant = new XboxController(1);
  private XboxController switchBoard = new XboxController(2);
  private XboxController phantom = new XboxController(3);

  // OI - operator inputs
  public JoystickButton heightDownSelect; // used in hunting/delivery modes
  public JoystickButton heightUpSelect; // used in hunting/delivery
  public JoystickButton captureRelease; // flips hunt/deliver mode
  public JoystickButton flip; // used to flip
  public JoystickButton endDriveMode; // Switches state out of drive
  public JoystickButton goToPrevMode; // Goes to previous state (only works for recapturing)

  public Trigger climbButton;
  public Trigger shortClimbButton;

  private ExpoShaper rotateShaper = new ExpoShaper(.7); // fairly flat curve

  final ClimberSubsystem climber;
  final ArmSubsystem arm;
  final OI m_oi;

  //@SuppressWarnings({ "resource", })
  public OI() {
    arm = RobotContainer.getSubsystem(ArmSubsystem.class);
    climber = RobotContainer.getSubsystemOrNull(ClimberSubsystem.class);
    m_oi = this;  

    // Wait until we get the first switchboard input - hack we know.
    try {
      Thread.sleep(250);
    } catch (InterruptedException e) {
      // don't care - won't happen
    }

    // THIS code was supposed to look at initial condition of sideboard and and select binding.
    // For now just select which bindings.
    // If the Test Button on the switchboard is active, select for TestBinding
    ///if (false/* switchBoard.getRawButton(11) */) {
    ///  bindTestButtons();
    ///  System.out.println("Using Test OI");
    ///} else 
      bindFieldButtons();
      System.out.println("Using Field OI");
    
  }

  private void bindFieldButtons() {
    // Drive Train Commands
    new JoystickButton(driver, XboxControllerButtonCode.B.getCode())
        .onTrue(new ToggleAutomaticGearShiftingCommand());
    new JoystickButton(driver, XboxControllerButtonCode.X.getCode()).onTrue(new InvertDriveControlsCommand());
    new JoystickButton(driver, XboxControllerButtonCode.LB.getCode()).whileTrue(new LimeLightArcadeDriveCommand(1.0));
    new JoystickButton(driver, XboxControllerButtonCode.RB.getCode()).onTrue(new AutomaticUpShiftCommand());
    new JoystickTrigger(driver, XboxControllerButtonCode.TRIGGER_LEFT.getCode(), 0.75)
        .whileTrue(new AutoCargoIntakeCommand(0.4));
    new JoystickTrigger(driver, XboxControllerButtonCode.TRIGGER_RIGHT.getCode(), 0.75)
        .whileTrue(new OuttakeTestCmd(0.4));
    new JoystickButton(assistant, 9).whileTrue(new CopilotControlCommand(0.4, 0.4));

    // Switchboard Assignments 
    /**
     * 
     * TODO: are these used???  Temp adding Climber tests buttons to live code to help debug CommandGroup
     * DPL - 3/23/19
    new JoystickButton(switchBoard, 1).onTrue(new DeployCargoTrapCommand());
    new JoystickButton(switchBoard, 2).onTrue(new RetractCargoTrapCommand());
    */

    // reset arm if lower limit swith is hit - compensate for hitting wall and skipping belt teeth - dpl 2025  
    new Trigger(arm::extensionAtMin).onTrue(new ResetArmCommand());

    // setup buttons for use in CommandManager
    heightDownSelect = new JoystickButton(assistant, XboxControllerButtonCode.LB.getCode());
    heightUpSelect = new JoystickButton(assistant, XboxControllerButtonCode.RB.getCode());
    captureRelease = new JoystickButton(assistant, XboxControllerButtonCode.A.getCode());
    flip = new JoystickButton(assistant, XboxControllerButtonCode.X.getCode());
    endDriveMode = new JoystickButton(assistant, XboxControllerButtonCode.B.getCode());
    goToPrevMode = new JoystickButton(assistant, XboxControllerButtonCode.Y.getCode());

    //TODO: Billy / Zander / driveteam pick a real place for this - 3/23/19
    climbButton = new Trigger(() -> switchBoard.getRawButton(7) && switchBoard.getRawButton(11));
    shortClimbButton = new Trigger(() -> switchBoard.getRawButton(8) && switchBoard.getRawButton(11));
  }


  /**
   *  Test Bindings - used if the hard coded flag above is set to TRUE see line#.
   */
  public void bindTestButtons() {
    // Vacuum subsystem tests
    // new JoystickButton(assistant,
    // XboxControllerButtonCode.B.getCode()).onTrue(new
    // SolenoidTestCommand(false));
    ///  whenPressed --> onTrue
    ///  whileHeld --> whileTrue
    new JoystickButton(assistant, XboxControllerButtonCode.A.getCode()).onTrue(new VacuumCommand(true, 2.0));
    new JoystickButton(assistant, XboxControllerButtonCode.B.getCode()).onTrue(new VacuumCommand(false, 2.0));

    // gearbox tests
    new JoystickButton(driver, XboxControllerButtonCode.A.getCode()).onTrue(new DownShiftCommand());
    new JoystickButton(driver, XboxControllerButtonCode.Y.getCode()).onTrue(new UpShiftCommand());
    new JoystickButton(driver, XboxControllerButtonCode.B.getCode()).whileTrue(new IntakeTestCmd(0.4));
    new JoystickButton(driver, XboxControllerButtonCode.X.getCode()).whileTrue(new OuttakeTestCmd(0.4));

    
    // setup buttons - required for Control Manager construction, but not really
    // used.
    heightDownSelect = new JoystickButton(phantom, XboxControllerButtonCode.LB.getCode());
    heightUpSelect = new JoystickButton(phantom, XboxControllerButtonCode.RB.getCode());
    captureRelease = new JoystickButton(phantom, XboxControllerButtonCode.Y.getCode());
    flip = new JoystickButton(phantom, XboxControllerButtonCode.X.getCode());
    endDriveMode = new JoystickButton(phantom, XboxControllerButtonCode.B.getCode());
  }

  // Bind analog controls to functions to use by the commands
  // this way we only change it key/stick assignemnts once.

  // Use Xbox Trigger buttons to directly make small adustments to the arm, raw stick units
  // converted in the CommandManager
  public double adjustHeight() {
    return m_oi.assistant.getLeftTriggerAxis()  //Hand.kLeft) 
         - m_oi.assistant.getRightTriggerAxis(); //Hand.kRight);
  }

  public double extensionInput() {
    return m_oi.assistant.getLeftY(); //Hand.kLeft);
  }

  // assistant rotation input
  public double rotationInput() {
    double in = m_oi.assistant.getRightY(); //Hand.kRight);
    double out = rotateShaper.expo(in);
    return out;
  }

  public XboxController getDriverController() {
    return driver;
  }

  public XboxController getAssistantController() {
    return assistant;
  }

  public XboxController getSwitchBoard() {
    return switchBoard;
  }
}