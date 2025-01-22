package frc.robot2019.commands.drive;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.subsystem.hid.ExpoShaper;
import frc.robot2019.OI;
import frc.robot2019.input.LimeLightXFilteredInput;
import frc.robot2019.subsystems.DriveTrainSubsystem;
import frc.robot2019.subsystems.SensorSubsystem;

/**
 * An example command. You can replace me with your own command.
 */
public class LimeLightArcadeDriveCommand extends Command {
  private final double P = 0.22;
  private final double I = 0.0;
  private final double D = 0.3;
  private PIDController controller;
  private ExpoShaper speedShaper;
  private double maxSpeed;

  final SensorSubsystem sensorSubsystem;
  final DriveTrainSubsystem driveTrain;
  final private OI m_oi;

  public LimeLightArcadeDriveCommand(double maxSpeed) {
    driveTrain = RobotContainer.getSubsystem(DriveTrainSubsystem.class);
    sensorSubsystem = RobotContainer.getSubsystem(SensorSubsystem.class);
    m_oi = RobotContainer.getObject("OI");
    // Use addRequirements() here to declare subsystem dependencies
    addRequirements(driveTrain);
    controller = new PIDController(P, I, D);
    var tbd =  new LimeLightXFilteredInput();
    
    speedShaper = new ExpoShaper(0.6); // 0 no change, 1.0 max flatness
    this.maxSpeed = maxSpeed;
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    controller.reset();
    controller.setInputRange(-25.0, 25.0);
    controller.setOutputRange(-0.5, 0.5);
    controller.setPercentTolerance(1);
    controller.setContinuous(true);
    controller.enable();
    sensorSubsystem.enableLED();
    execute();
  }

  // Read Controller Input from two joysticks.
  // Left joystick controls the left motors and the right joystick controls the
  // right motors
  // Temporary until we get the XboxController wrapper for joystick
  @Override
  public void execute() {
    // We invert the PID controller value so the feedback loop is negative and not
    // positive
    double speed = maxSpeed * speedShaper.expo(m_oi.getDriverController().getLeftY()); // Hand.kLeft));
    double rotation = -controller.get();
 
    if (Math.abs(rotation) <= 0.12) {
      rotation = Math.signum(rotation) * 0.12;
    }

    SmartDashboard.putNumber("PID Error", controller.getError());

    driveTrain.ArcadeDrive(speed, rotation, true);
    SmartDashboard.putData(controller);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    sensorSubsystem.disableLED();
    controller.reset();
    driveTrain.stop();
  }
}