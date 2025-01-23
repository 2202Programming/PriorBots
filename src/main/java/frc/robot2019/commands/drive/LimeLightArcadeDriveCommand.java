package frc.robot2019.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.subsystem.hid.ExpoShaper;
import frc.robot2019.OI;
import frc.robot2019.subsystems.DriveTrainSubsystem;
import frc.robot2019.subsystems.SensorSubsystem;

public class LimeLightArcadeDriveCommand extends Command {
  final double P = 0.22;
  final double I = 0.0;
  final double D = 0.3;
  final PIDController controller;
  final ExpoShaper speedShaper;
  final double maxSpeed;

  // LL will use a simple avg filter filter, typically 3-5 tap avg
  final LinearFilter lowPassFilter;

  final SensorSubsystem sensorSubsystem;
  final DriveTrainSubsystem driveTrain;
  final private OI m_oi;

  public LimeLightArcadeDriveCommand(double maxSpeed) {
    this(maxSpeed, 5);
  }

  public LimeLightArcadeDriveCommand(double maxSpeed, int num_taps) {
    driveTrain = RobotContainer.getSubsystem(DriveTrainSubsystem.class);   // ss we are controlling
    sensorSubsystem = RobotContainer.getSubsystem(SensorSubsystem.class);  // access to LL X
    m_oi = RobotContainer.getObject("OI");                                // access stick (older style)
    // Use addRequirements() here to declare subsystem dependencies
    addRequirements(driveTrain);

    // filters noise on LL signal, at expense of slower response
    lowPassFilter = LinearFilter.movingAverage(num_taps);
    lowPassFilter.reset();

    // PID will control rotation based on limelight
    controller = new PIDController(P, I, D);
    // driver will control speed via shaped stick inputs
    speedShaper = new ExpoShaper(0.6); // 0 no change, 1.0 max flatness
    this.maxSpeed = maxSpeed;
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    /*
     * old settings on PID controller -left for future tuning
     * controller.setInputRange(-25.0, 25.0);
     * controller.setOutputRange(-0.5, 0.5);
     * controller.setPercentTolerance(1);
     * controller.setContinuous(true);
     * controller.enable();
     */
    sensorSubsystem.enableLED();
    controller.reset();
    lowPassFilter.reset();
    controller.setSetpoint(0.0); // LL goal is to zero rotation on target
  }

  @Override
  public void execute() {
    double X = sensorSubsystem.getX();
    double X_filtered = lowPassFilter.calculate(X);
    // We invert the PID controller value so the feedback loop is negative and not
    // positive
    double speed = maxSpeed * speedShaper.expo(m_oi.getDriverController().getLeftY()); // Hand.kLeft));
    double rotation = -controller.calculate(X_filtered); //

    // DPL - not sure what this was doing, but I thing the <= is wrong
    // Changed to make this a max rotation rate cap
    if (Math.abs(rotation) >= 0.12) { // was <= 0.12, changed to >=
      rotation = Math.signum(rotation) * 0.12;
    }

    driveTrain.ArcadeDrive(speed, rotation, true);
    SmartDashboard.putData(controller);
    SmartDashboard.putNumber("PID Error", controller.getError());
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