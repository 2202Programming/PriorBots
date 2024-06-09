package frc.robot2023.commands.Arm;



import edu.wpi.first.wpilibj2.command.Command;
import frc.base.RobotContainerOrig;
import frc.robot2023.subsystems.Elbow;

public class MoveElbow extends Command {
  final Elbow elbow = RobotContainerOrig.RC().elbow;
  final double angle;
  final double maxVel;

  double old_maxVel;
 
  /** Creates a new GamePieceAngle. */
  public MoveElbow(double angle) {
    this(angle, -1.0);
  }
  public MoveElbow(double angle, double maxVel)  {
    this.angle = angle;
    this.maxVel = (maxVel < 0.0) ? elbow.getMaxVel() : maxVel;
    addRequirements(elbow);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    old_maxVel = elbow.getMaxVel();
    elbow.setSetpoint(angle); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // nothing to do, but wait for the wrist to move
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elbow.setMaxVel(old_maxVel);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elbow.atSetpoint();
  }
}
