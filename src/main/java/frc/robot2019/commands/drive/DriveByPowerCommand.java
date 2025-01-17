package frc.robot2019.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot2019.Robot;
import frc.robot2019.subsystems.DriveTrainSubsystem;

/**
 * An example command. You can replace me with your own command.
 */
public class DriveByPowerCommand extends Command {
  private DriveTrainSubsystem driveTrain = Robot.driveTrain;
  double power;
  double timeout;

  public DriveByPowerCommand(double power, double timeout) {
    this.power = power;
    this.timeout = timeout;
    // Use addRequirements() here to declare subsystem dependencies
    addRequirements(Robot.driveTrain);
  }
  
  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    //  driveTrain.stop();
    // may want to check counters... if we try to control this...
    setTimeout(timeout);
  }

  // Read Controller Input from two joysticks.
  // Left joystick controls the left motors and the right joystick controls the
  // right motors
  // Temporary until we get the XboxController wrapper for joystick
  @Override
  public void execute() {
    Robot.driveTrain.ArcadeDrive(power, 0.0, true);
  }

  @Override
  public boolean isFinished() {
    return isTimedOut();
  }

  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
  }

  @Override
  public void interrupted() {
  }
}