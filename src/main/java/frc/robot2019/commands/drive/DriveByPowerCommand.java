package frc.robot2019.commands.drive;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib2202.builder.RobotContainer;
import frc.robot2019.subsystems.DriveTrainSubsystem;

/**
 * An example command. You can replace me with your own command.
 */
public class DriveByPowerCommand extends WaitCommand {
  final private DriveTrainSubsystem driveTrain;
  double power;

  public DriveByPowerCommand(double power, double timeout) {
    super(timeout);
    this.power = power;    
    driveTrain = RobotContainer.getSubsystem(DriveTrainSubsystem.class);
    // Use addRequirements() here to declare subsystem dependencies
    addRequirements(driveTrain);
  }
  
  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    //  driveTrain.stop();
    // may want to check counters... if we try to control this...
    super.initialize();  // was setTimeout(timeout);
  }

  // Read Controller Input from two joysticks.
  // Left joystick controls the left motors and the right joystick controls the
  // right motors
  // Temporary until we get the XboxController wrapper for joystick
  @Override
  public void execute() {
   driveTrain.ArcadeDrive(power, 0.0, true);
  }

  @Override
  public boolean isFinished() {
    return super.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
  }

}