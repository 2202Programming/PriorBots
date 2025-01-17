package frc.robot2019.commands.drive;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot2019.Robot;
import frc.robot2019.commands.util.ExpoShaper;
import frc.robot2019.subsystems.DriveTrainSubsystem;
/**
 * An example command. You can replace me with your own command.
 */
public class CopilotControlCommand extends Command {
  private DriveTrainSubsystem driveTrain;
  private ExpoShaper speedShaper;
  private ExpoShaper rotationShaper;
  private double maxSpeed;
  private double maxRotation;

  public CopilotControlCommand(double maxSpeed, double maxRotation) {
    // Use addRequirements() here to declare subsystem dependencies
    addRequirements(Robot.driveTrain);
    driveTrain = Robot.driveTrain;
    this.maxSpeed = maxSpeed;
    this.maxRotation = maxRotation;

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
    //Robot.driveTrain.ArcadeDrive(0.90, 0, true);
    double s = maxSpeed * speedShaper.expo(Robot.m_oi.getAssistantController().getY(Hand.kRight));
    //soften the input by limiting the max input
    double rot = maxRotation * rotationShaper.expo(0.8 * Robot.m_oi.getAssistantController().getX(Hand.kRight));
    Robot.driveTrain.ArcadeDrive(s, rot, false);
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