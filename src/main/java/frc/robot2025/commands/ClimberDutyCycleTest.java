package frc.robot2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2202.builder.RobotContainer;
import frc.robot2025.subsystems.Climber;

public class ClimberDutyCycleTest  extends Command {
    final Climber climber;
    double duty;

    public ClimberDutyCycleTest(double duty) {
      climber = RobotContainer.getSubsystem(Climber.class);
      this.duty = duty;
      addRequirements(climber);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {    
      climber.setDutyCycleMode(duty);
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
     climber.setDutyCycleMode(0.0);  //returns to servo mode
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false; // cmd runs until driver releases button
    }
  
  }