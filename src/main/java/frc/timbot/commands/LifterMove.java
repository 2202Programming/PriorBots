package frc.timbot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.lib2202.command.WatcherCmd;
import frc.robot2025.subsystems.Elevator_Subsystem.Levels;
import frc.timbot.subsystem.ShooterLifter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LifterMove extends Command {

  final ShooterLifter m_lifter; // works entirely in cm
  double h_cmd;
  final double h_max = 6 * 2.54;
  final double h_min = 0.0;

  public LifterMove(double h_cmd) {
    m_lifter = RobotContainer.getSubsystem(ShooterLifter.class);
    this.h_cmd = h_cmd;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_lifter.setHeight(h_cmd);
  }

  

  public void end(boolean interrupt){
    // if (interrupt) {
    //   m_lifter.setHeight(0);
    // }
  }
 

  @Override
  public boolean isFinished() {
    return m_lifter.isAtPosition();
  }

}
