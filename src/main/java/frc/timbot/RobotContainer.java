// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.PCM;
import frc.robot.Constants.CAN;
import frc.robot.commands.AdjustElevation;
import frc.robot.commands.Fire;
import frc.robot.commands.FireThenIdle;
import frc.robot.commands.SetSpinFlywheel;
import frc.robot.commands.SetSpinFlywheel;
import frc.robot.subsystems.Elevation;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Trigger;
import frc.robot.utils.hid.DriverControls;
import frc.robot.utils.hid.XboxButton;
import frc.robot.utils.hid.DriverControls.Id;

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
    dc.bind(Id.Driver, XboxButton.A).whileHeld(new SetSpinFlywheel(m_flywheel, 0.5))
        .whenReleased(new SetSpinFlywheel(m_flywheel, 0));

    dc.bind(Id.Driver, XboxButton.X).whenPressed(new AdjustElevation(m_elevator)); // increments angle by 50

    dc.bind(Id.Driver, XboxButton.B).whenPressed(new FireThenIdle(m_trigger, m_flywheel));
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
