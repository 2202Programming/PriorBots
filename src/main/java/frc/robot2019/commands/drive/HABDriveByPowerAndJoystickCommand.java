package frc.robot2019.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.subsystem.hid.ExpoShaper;
import frc.robot2019.OI;
import frc.robot2019.subsystems.ClimberSubsystem;
import frc.robot2019.subsystems.DriveTrainSubsystem;

/**
 * An example command. You can replace me with your own command.
 */
public class HABDriveByPowerAndJoystickCommand extends Command {
  final private DriveTrainSubsystem driveTrain;
  final ClimberSubsystem climber;
  final OI m_oi;
  double power;
  double minPower;
  double maxPower;
  double timeout;
  private ExpoShaper speedShaper;
  private ExpoShaper rotationShaper;

  public HABDriveByPowerAndJoystickCommand(double power, double minPower, double maxPower) {
    driveTrain = RobotContainer.getSubsystem(DriveTrainSubsystem.class);
    climber = RobotContainer.getSubsystemOrNull(ClimberSubsystem.class);  // could be null, deal
    m_oi = RobotContainer.getObject("OI");
    this.power = power;
    this.minPower = minPower;
    this.maxPower = maxPower;
    this.speedShaper = new ExpoShaper(0.6);        //0 no change,  1.0 max flatness
    this.rotationShaper = new ExpoShaper(0.5);
    // Use addRequirements() here to declare subsystem dependencies
    addRequirements(driveTrain);
  }
  
  // Read Controller Input from two joysticks.
  // Left joystick controls the left motors and the right joystick controls the
  // right motors
  // Temporary until we get the XboxController wrapper for joystick
  @Override
  public void execute() {
    double speedInput = speedShaper.expo(m_oi.getDriverController().getLeftY()); //Hand.kLeft));
    double speedAdjust = speedInput > 0? speedInput * (maxPower - power): speedInput * (power - minPower);
    double rotation = 0.5 * rotationShaper.expo(m_oi.getDriverController().getRightX()); //Hand.kRight));
    driveTrain.ArcadeDrive(power + speedAdjust, rotation, true);
  }

  @Override
  public boolean isFinished() {
    if (climber != null)
      return climber.climberAgainstWall();
    else return true;
  }

  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
  }

}