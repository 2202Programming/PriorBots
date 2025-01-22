package frc.robot2019.commands.drive;

import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2019.OI;
import frc.robot2019.subsystems.DriveTrainSubsystem;

/**
 * An example command. You can replace me with your own command.
 */
public class TankDriveCommand extends Command {
  final private DriveTrainSubsystem driveTrain;
  final private XboxController ctrl;
  final OI m_oi;

  public TankDriveCommand() {
    driveTrain = RobotContainer.getSubsystem(DriveTrainSubsystem.class);
    m_oi = RobotContainer.getObject("OI");
    ctrl = m_oi.getDriverController();
    // Use addRequirements() here to declare subsystem dependencies
    addRequirements(driveTrain);
  }
  
  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    driveTrain.stop();
  }

  // Read Controller Input from two joysticks.
  // Left joystick controls the left motors and the right joystick controls the
  // right motors
  // Temporary until we get the XboxController wrapper for joystick
  @Override
  public void execute() {
    driveTrain.tankDrive(ctrl.getLeftY(), ctrl.getRightY(), true);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
  }

}