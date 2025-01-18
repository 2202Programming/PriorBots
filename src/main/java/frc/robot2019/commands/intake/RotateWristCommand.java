package frc.robot2019.commands.intake; 

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2019.subsystems.IntakeSubsystem;


/**
 * 
 * Use a timeout to wait, so a "AND" test is performed because
 * the actual servo has now way of reporting back to Rio that
 * we aren't at the desired postion.  Time out gives it time to get there.
 * 
 */
public class RotateWristCommand extends Command{
    private double angle;
    private double timeout;
    final IntakeSubsystem intake;
    final Timer timer;

    public RotateWristCommand(double angle, double timeout){
        intake = RobotContainer.getSubsystem(IntakeSubsystem.class);
        timer = new Timer();
        addRequirements(intake);
        this.angle = angle;
        this.timeout = timeout;
    }

    public RotateWristCommand(double angle) {
        this(angle, 0.0);
    }

    @Override
    public void initialize() {
        timer.start(); // setTimeout(timeout); 
        intake.setAngle(angle);
    }

  @Override
  public void execute() {      
  }

  @Override
  public boolean isFinished() {
     // position in 1.0 degrees, but servo will always hit.
    double measAngle = intake.getAngle();
    boolean posHit = ( Math.abs(measAngle - angle) < 1.0 );
     //stay for the whole timeout
     return (timer.hasElapsed(timeout) && posHit);
  }

  @Override
  public void end(boolean interrupted) {
  }

}

