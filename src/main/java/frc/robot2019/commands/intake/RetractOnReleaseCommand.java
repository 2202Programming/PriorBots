package frc.robot2019.commands.intake;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2019.commands.CommandManager;
import frc.robot2019.commands.arm.ArmStatePositioner;
import frc.robot2019.subsystems.ArmSubsystem;
import frc.robot2019.subsystems.IntakeSubsystem;

public class RetractOnReleaseCommand extends Command {
  BooleanSupplier releaseCheckFunc;
  double x_retract;
  double timeout;
  double init_x;
  double init_h;
  double x_new; // x to jump to when the sensor says we are released

  final ArmSubsystem arm;
  final ArmStatePositioner armPositioner;
  final IntakeSubsystem intake;
  final Timer timer;

  public RetractOnReleaseCommand(CommandManager cmdMgr, double x_retract, double timeout) {
    intake = RobotContainer.getSubsystem(IntakeSubsystem.class);
    arm = RobotContainer.getSubsystem(ArmSubsystem.class);
    armPositioner = arm.getArmPositioner();

    // could require(vacSensor0)
    this.releaseCheckFunc = intake.getVacuumSensor()::hasReleased;
    this.x_retract = x_retract;
    this.timeout = timeout;
    this.timer = new Timer();
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    // save were we are so we can tweek it on finish
    init_x = armPositioner.getProjectionCommanded();
    init_h = armPositioner.getHeightCommanded();
    x_new = init_x;
    int invertMultiplier = arm.isInverted() ? -1 : 1;
    x_new -= invertMultiplier * x_retract; // move back a bit, account for side.

    timer.start(); //setTimeout(timeout);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    // DEBUG CODE
    boolean rc = releaseCheckFunc.getAsBoolean();
    SmartDashboard.putBoolean("ReleaseSensor", rc);
    // we are done, we timed out or we got the vacuum release signal, move us back.
    // RC is good, move arm back now
    if (rc) {
      armPositioner.getDriverAdjustLimiter().setX(0.0);
      armPositioner.setPosition(init_h, x_new);
    }

  }

  boolean checkArmPos() {
    double curX_proj_err = Math.abs(arm.getProjection() - x_new);
    return (curX_proj_err < .25) || arm.isExtensionOverrided(); // .25 inch
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(timeout) || checkArmPos();
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {  }

}
