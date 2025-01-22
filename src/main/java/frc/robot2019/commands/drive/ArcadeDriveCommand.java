package frc.robot2019.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.subsystem.hid.ExpoShaper;
import frc.robot2019.OI;
import frc.robot2019.subsystems.DriveTrainSubsystem;
/**
 * An example command. You can replace me with your own command.
 */
public class ArcadeDriveCommand extends Command {
  final private DriveTrainSubsystem driveTrain;
  final private OI m_oi;
  private ExpoShaper speedShaper;
  private ExpoShaper rotationShaper;

  public ArcadeDriveCommand() {
    driveTrain = RobotContainer.getSubsystem(DriveTrainSubsystem.class);
    m_oi = RobotContainer.getObject("OI");
    // Use addRequirements() here to declare subsystem dependencies
    addRequirements(driveTrain);

    speedShaper = new ExpoShaper(0.6);        //0 no change,  1.0 max flatness
    rotationShaper = new ExpoShaper(0.5);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    execute();
  }

  // Read Controller Input from two joysticks.
  // Left joystick controls the left motors and the right joystick controls the
  // right motors
  // Temporary until we get the XboxController wrapper for joystick
  @Override
  public void execute() {
    //driveTrain.ArcadeDrive(0.90, 0, true);
    double s = speedShaper.expo(m_oi.getDriverController().getLeftY()); //Hand.kLeft));
    //soften the input by limiting the max input
    double rot = rotationShaper.expo(0.8 * m_oi.getDriverController().getRightX()); //Hand.kRight));
    driveTrain.ArcadeDrive(s, rot, false);
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