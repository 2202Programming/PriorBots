package frc.robot2019.commands.drive.shift;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2019.OI;
import frc.robot2019.subsystems.DriveTrainSubsystem;

/**
 * An example command. You can replace me with your own command.
 */
public class ThrottleCommand extends Command {
  private final double CYCLE_TIME_IN_SECONDS = 0.020;
  private int cycleCount;
  private int maxCycles;
  private double stepValue;
  private double startValue;

  final private DriveTrainSubsystem driveTrain;
  final OI m_oi;

  /**
   * 
   * @param rampTime Ramp up time in seconds
   */
  public ThrottleCommand(double rampTime, double startValue, double endValue) {
    driveTrain = RobotContainer.getSubsystem(DriveTrainSubsystem.class);
    m_oi = RobotContainer.getObject("OI");
    // Use addRequirements() here to declare subsystem dependencies
    addRequirements(driveTrain);
    maxCycles = (int) Math.ceil(rampTime / CYCLE_TIME_IN_SECONDS);
    this.startValue = startValue;
    stepValue = (endValue - startValue) / maxCycles;
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    cycleCount = 0;
    execute();
  }

  // Read Controller Input from two joysticks.
  // Left joystick controls the left motors and the right joystick controls the
  // right motors
  @Override
  public void execute() {
    double throttle = m_oi.getDriverController().getLeftY(/*Hand.kLeft*/) * (startValue + stepValue * cycleCount);
    double turnRate = m_oi.getDriverController().getRightX(/*Hand.kRight*/) * (startValue + stepValue * cycleCount);
    cycleCount++;
    driveTrain.ArcadeDrive(throttle, turnRate, true);
  }

  @Override
  public boolean isFinished() {
    double leftSpeed = Math.abs(driveTrain.getLeftEncoderTalon().getSelectedSensorVelocity());
    double rightSpeed = Math.abs(driveTrain.getRightEncoderTalon().getSelectedSensorVelocity());
    double curSpeed = (leftSpeed + rightSpeed) / 2.0;
    double shiftSpeed = AutomaticGearShiftCommand.DOWNSHIFT_SPEED_LOW * AutomaticGearShiftCommand.MAXSPEED_IN_COUNTS_PER_SECOND;
    return cycleCount >= maxCycles || curSpeed < shiftSpeed;
  }

  @Override
  public void end(boolean interrupted) {
  }
}