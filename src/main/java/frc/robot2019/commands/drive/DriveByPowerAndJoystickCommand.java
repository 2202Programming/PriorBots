package frc.robot2019.commands.drive;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot2019.Robot;
import frc.robot2019.commands.util.ExpoShaper;
import frc.robot2019.subsystems.DriveTrainSubsystem;

/**
 * An example command. You can replace me with your own command.
 */
public class DriveByPowerAndJoystickCommand extends Command {
  private DriveTrainSubsystem driveTrain = Robot.driveTrain;
  double power;
  double minPower;
  double maxPower;
  double timeout;
  private ExpoShaper speedShaper;
  private ExpoShaper rotationShaper;

  public DriveByPowerAndJoystickCommand(double power, double minPower, double maxPower, double timeout) {
    this.power = power;
    this.minPower = minPower;
    this.maxPower = maxPower;
    this.timeout = timeout;
    this.speedShaper = new ExpoShaper(0.6);        //0 no change,  1.0 max flatness
    this.rotationShaper = new ExpoShaper(0.5);
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
    double speedInput = speedShaper.expo(Robot.m_oi.getDriverController().getY(Hand.kLeft));
    double speedAdjust = speedInput > 0? speedInput * (maxPower - power): speedInput * (power - minPower);
    double rotation = 0.5 * rotationShaper.expo(Robot.m_oi.getDriverController().getX(Hand.kRight));
    Robot.driveTrain.ArcadeDrive(power + speedAdjust, rotation, true);
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