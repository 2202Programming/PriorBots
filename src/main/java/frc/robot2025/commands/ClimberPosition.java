package frc.robot2025.commands;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2025.subsystems.Climber;

public class ClimberPosition extends Command {
  final Climber climber;
  final double maxVel;
  final double pos;

  public ClimberPosition(double position) {
    this(position, 5.0); // pick a slow speed for testing if this api used
  }

  public ClimberPosition(double position, double maxVel) {
    //SmartDashboard.putNumber("Climber/Desired Pos", position);
    climber = RobotContainer.getSubsystem(Climber.class);
    this.pos = position;
    this.maxVel = maxVel;  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //double cmdPos = SmartDashboard.getNumber("Climber/Desired Pos", 0.0);
    climber.setMaxVelocity(maxVel);
    climber.setSetpoint(pos);
  }

  // Called once the command ends or is interrupted.
  //@Override
  //public void end(boolean interrupted) {
    //climber.setVelocity(0.0);  Don't need to do this, servo stays in positon mode
  //}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   return climber.atSetpoint();
  }
}
