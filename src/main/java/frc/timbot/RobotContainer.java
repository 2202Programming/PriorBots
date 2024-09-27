// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.timbot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.timbot.Constants.PCM;
import frc.timbot.Constants.CAN;
import frc.timbot.commands.AdjustElevation;
import frc.timbot.commands.Fire;
import frc.timbot.commands.FireThenIdle;
import frc.timbot.commands.SetSpinFlywheel;
import frc.timbot.subsystems.Elevation;
import frc.timbot.subsystems.Flywheel;
import frc.timbot.subsystems.Trigger;
import frc.timbot.utils.hid.DriverControls;
import frc.timbot.utils.hid.XboxButton;
import frc.timbot.utils.hid.DriverControls.Id;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private TalonFX motor1;
  private TalonFX motor2;
  private TalonSRX actuator;
  private DoubleSolenoid solenoid;

  private Flywheel m_flywheel;
  private Trigger m_trigger;
  private Elevation m_elevator;

  private DriverControls dc = new DriverControls();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    motor1 = new TalonFX(Constants.CAN.FLYWHEEL_TALON1);
    motor2 = new TalonFX(Constants.CAN.FLYWHEEL_TALON2);
    actuator = new TalonSRX(Constants.CAN.ACTUATOR_TALON);
    solenoid = new DoubleSolenoid(CAN.PCM, PCM.TRIGGER_FORWARD, PCM.TRIGGER_BACK);

    m_flywheel = new Flywheel(motor1, motor2);
    m_trigger = new Trigger(solenoid);
    m_elevator = new Elevation(actuator);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    dc.registerController(Id.Driver, new XboxController(1));
    dc.bind(Id.Driver, XboxButton.A).whileTrue(new SetSpinFlywheel(m_flywheel, 0.5))
        .button(new SetSpinFlywheel(m_flywheel, 0));

    dc.bind(Id.Driver, XboxButton.X).onTrue(new AdjustElevation(m_elevator)); // increments angle by 50

    dc.bind(Id.Driver, XboxButton.B).onTrue(new FireThenIdle(m_trigger, m_flywheel));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
